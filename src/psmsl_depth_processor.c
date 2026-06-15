/**
 * @file psmsl_depth_processor.c
 * @brief PSMSL-Inspired Spatial Lattice Processor for Subnet 124
 *
 * Translates point cloud arrays into high-performance semantic grid indexes.
 * Copyright (c) 2026 - All Rights Reserved
 */

#include "psmsl_depth_processor.h"
#include <math.h>
#include <string.h>

// ── Strict Local Resolution Parameters (MCU Optimised) ─────────────────────
#define Resolution_XY               0.40f   // Spatial Grid Cell Width (m)
#define Resolution_Z                0.80f   // Height Bin Span Thickness (m)
#define Inv_Resolution_XY           2.50f   // Pre-inverted multiplication factor
#define Inv_Resolution_Z            1.25f   // Pre-inverted height shift factor

/**
 * @brief Maps high-precision world coordinates to localized discrete array coordinates.
 * Eliminates slow floating-point division using pre-inverted scaling constants.
 */
static inline void world_to_grid(const float world_pos[3], const float origin[3], int grid_coords[3]) 
{
    grid_coords[0] = (int)floorf((world_pos[0] - origin[0]) * Inv_Resolution_XY);
    grid_coords[1] = (int)floorf((world_pos[1] - origin[1]) * Inv_Resolution_XY);
    grid_coords[2] = (int)floorf((world_pos[2] - origin[2]) * Inv_Resolution_Z);
}

/**
 * @brief Executes rapid spatial-coherence and navigability grading over downsampled points.
 * Bypasses high-overhead Python spatial clustering algorithms natively.
 */
void psmsl_depth_analyze_obstacles(const float obstacles[][4], int num_obstacles,
                                   const float current_pos[3], const float current_rpy[3],
                                   float search_radius,
                                   psmsl_depth_result_t *result)
{
    if (!result) return;

    // Reset outbound memory structure state
    result->clutter_density = 0.0f;
    result->local_navigability = 1.0f; 
    result->collision_risk_score = 0.0f;

    if (num_obstacles <= 0 || obstacles == NULL) {
        return;
    }

    // Establish bounding box corners centered tightly around the drone axis
    float grid_origin[3] = {
        current_pos[0] - ((float)PSMSL_DEPTH_MAX_GRID_DIM * Resolution_XY * 0.5f),
        current_pos[1] - ((float)PSMSL_DEPTH_MAX_GRID_DIM * Resolution_XY * 0.5f),
        current_pos[2] - ((float)PSMSL_DEPTH_MAX_HEIGHT_BINS * Resolution_Z * 0.5f)
    };

    // Statically allocated 1D occupancy representation array (Avoids runtime heap allocations)
    int density_grid[PSMSL_DEPTH_MAX_GRID_DIM * PSMSL_DEPTH_MAX_GRID_DIM * PSMSL_DEPTH_MAX_HEIGHT_BINS];
    memset(density_grid, 0, sizeof(density_grid));

    int obstacles_in_radius = 0;
    int unique_occupied_cells = 0;
    float total_proximity_weight = 0.0f;
    float min_dist_to_obstacle = search_radius + 1.0f;

    // ── Single-Pass Spatial Filtering Scan ──
    for (int i = 0; i < num_obstacles; ++i) {
        float ox = obstacles[i][0];
        float oy = obstacles[i][1];
        float oz = obstacles[i][2];
        float or = obstacles[i][3]; // Native bounding sphere safety shell offset

        // Drop unprojected space points or raw sensor initialization blanks
        if (oz >= 990.0f || (ox == 0.0f && oy == 0.0f && oz == 0.0f)) {
            continue;
        }

        float dx = ox - current_pos[0];
        float dy = oy - current_pos[1];
        float dz = oz - current_pos[2];
        float dist = sqrtf(dx * dx + dy * dy + dz * dz + 1e-8f);
        float effective_dist = dist - or;

        // Bounded Proximity Check
        if (effective_dist < search_radius) {
            obstacles_in_radius++;
            
            // Proximity grading weighting (Closer items generate linear penalization spikes)
            total_proximity_weight += (1.0f - (fmaxf(effective_dist, 0.0f) / search_radius));

            if (effective_dist < min_dist_to_obstacle) {
                min_dist_to_obstacle = effective_dist;
            }

            // Map world-frame cloud coordinates to local discrete array bins
            int grid_coords[3];
            float mapped_point[3] = {ox, oy, oz};
            world_to_grid(mapped_point, grid_origin, grid_coords);

            // Fast Array Boundary Intercept Check
            if (grid_coords[0] >= 0 && grid_coords[0] < PSMSL_DEPTH_MAX_GRID_DIM &&
                grid_coords[1] >= 0 && grid_coords[1] < PSMSL_DEPTH_MAX_GRID_DIM &&
                grid_coords[2] >= 0 && grid_coords[2] < PSMSL_DEPTH_MAX_HEIGHT_BINS) {
                
                int index = grid_coords[2] * (PSMSL_DEPTH_MAX_GRID_DIM * PSMSL_DEPTH_MAX_GRID_DIM) +
                            grid_coords[1] * PSMSL_DEPTH_MAX_GRID_DIM +
                            grid_coords[0];
                
                // Track spatial clustering coherence metrics
                if (density_grid[index] == 0) {
                    unique_occupied_cells++;
                }
                density_grid[index]++;
            }
        }
    }

    // ── Structural Navigability Assessment ──
    if (obstacles_in_radius > 0) {
        float total_cells = (float)(PSMSL_DEPTH_MAX_GRID_DIM * PSMSL_DEPTH_MAX_GRID_DIM * PSMSL_DEPTH_MAX_HEIGHT_BINS);
        float volumetric_occupancy = (float)unique_occupied_cells / total_cells;
        float proximity_density = total_proximity_weight / (float)obstacles_in_radius;

        // Blend proximity peaks with spatial volumetric allocation
        result->clutter_density = 0.7f * proximity_density + 0.3f * volumetric_occupancy;
        if (result->clutter_density > 1.0f) result->clutter_density = 1.0f;
    }

    // Calculate local spatial navigability
    result->local_navigability = 1.0f - result->clutter_density;
    if (result->local_navigability < 0.0f) result->local_navigability = 0.0f;

    // Linear Velocity Predictive Risk Evaluation
    if (min_dist_to_obstacle < search_radius) {
        float base_risk = 1.0f - (fmaxf(min_dist_to_obstacle, 0.0f) / search_radius);
        
        // Critical Threshold Breached (drone has entered immediate obstacle shell)
        if (min_dist_to_obstacle < 0.40f) {
            base_risk = fmaxf(base_risk, 0.95f);
        }
        result->collision_risk_score = base_risk;
    }
}

void psmsl_depth_analyze_image(const float *depth_image, int width, int height,
                               const float current_pos[3], const float current_rpy[3],
                               float max_range, float fov_deg, float search_radius,
                               psmsl_depth_result_t *result)
{
    if (!result || !depth_image) return;

    // Subsample the depth image to reduce MCU load (e.g., 16x16 grid)
    int stride_y = height / 16;
    if (stride_y < 1) stride_y = 1;
    int stride_x = width / 16;
    if (stride_x < 1) stride_x = 1;

    // Camera geometry
    float fov_rad = fov_deg * (M_PI / 180.0f);
    float focal_length = (width / 2.0f) / tanf(fov_rad / 2.0f);
    
    // Drone rotation matrix (Roll, Pitch, Yaw)
    float roll = current_rpy[0];
    float pitch = current_rpy[1];
    float yaw = current_rpy[2];
    
    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw), sy = sinf(yaw);
    
    // R = Rz * Ry * Rx
    float R[3][3];
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;

    // Temporary buffer for extracted obstacles (up to 256 points)
    #define MAX_EXTRACTED_OBS 256
    float extracted_obs[MAX_EXTRACTED_OBS][4];
    int num_extracted = 0;

    float min_depth_norm = 0.5f / max_range;

    for (int v = 0; v < height; v += stride_y) {
        for (int u = 0; u < width; u += stride_x) {
            if (num_extracted >= MAX_EXTRACTED_OBS) break;

            float d_norm = depth_image[v * width + u];
            
            // Ignore background/sky and points too close to drone body
            if (d_norm >= 0.99f || d_norm <= min_depth_norm) continue;

            float z_cam = d_norm * max_range;
            float x_cam = (u - width / 2.0f) * z_cam / focal_length;
            float y_cam = (v - height / 2.0f) * z_cam / focal_length;

            // Camera frame to Drone frame mapping:
            // Camera +Z -> Drone +X (forward)
            // Camera +X -> Drone -Y (right)
            // Camera +Y -> Drone -Z (down)
            float p_drone[3] = {z_cam, -x_cam, -y_cam};

            // Apply rotation and translation
            float p_world[3];
            p_world[0] = current_pos[0] + R[0][0]*p_drone[0] + R[0][1]*p_drone[1] + R[0][2]*p_drone[2];
            p_world[1] = current_pos[1] + R[1][0]*p_drone[0] + R[1][1]*p_drone[1] + R[1][2]*p_drone[2];
            p_world[2] = current_pos[2] + R[2][0]*p_drone[0] + R[2][1]*p_drone[1] + R[2][2]*p_drone[2];

            // Radius based on distance (points further away represent larger areas)
            float radius = z_cam * tanf(fov_rad / width * stride_x);
            if (radius < 0.2f) radius = 0.2f;

            extracted_obs[num_extracted][0] = p_world[0];
            extracted_obs[num_extracted][1] = p_world[1];
            extracted_obs[num_extracted][2] = p_world[2];
            extracted_obs[num_extracted][3] = radius;
            num_extracted++;
        }
    }

    // Now pass the extracted point cloud to the existing spatial analyzer
    psmsl_depth_analyze_obstacles(extracted_obs, num_extracted, current_pos, current_rpy, search_radius, result);
}
