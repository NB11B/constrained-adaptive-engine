/**
 * @file psmsl_depth_processor.h
 * @brief PSMSL-inspired depth processing for obstacle semantic analysis in flight control.
 *
 * This module processes raw obstacle point cloud data to extract higher-level
 * semantic information about the environment, such as clutter density, local
 * navigability, and potential collision risks, specifically tailored for
 * drone navigation.
 */

#ifndef PSMSL_DEPTH_PROCESSOR_H
#define PSMSL_DEPTH_PROCESSOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Configuration Constants
// =============================================================================

#define PSMSL_DEPTH_GRID_RESOLUTION 0.5f    // Resolution of the internal grid for density calculation (meters)
#define PSMSL_DEPTH_MAX_GRID_DIM    20      // Max grid dimension (e.g., 20x20 cells in XY plane)
#define PSMSL_DEPTH_MAX_HEIGHT_BINS 5       // Number of height bins for 3D density

// =============================================================================
// Datatypes & Enumerations
// =============================================================================

/**
 * @brief Structure to hold PSMSL-inspired depth analysis results.
 */
typedef struct {
    float clutter_density;          // Normalized value indicating how dense obstacles are (0.0 to 1.0)
    float local_navigability;       // Normalized value indicating ease of movement (0.0 to 1.0)
    float collision_risk_score;     // Aggregated score indicating immediate collision threat (0.0 to 1.0)
    // Add more semantic metrics as needed for flight control
} psmsl_depth_result_t;

// =============================================================================
// Public Functional Synapse Protocols
// =============================================================================

/**
 * @brief Processes obstacle data to derive PSMSL-inspired semantic information for flight control.
 *
 * This function takes a point cloud of obstacles and the drone's current state
 * to compute semantic metrics like clutter density and navigability.
 *
 * @param obstacles         Array of obstacle points [x, y, z, r].
 * @param num_obstacles     Number of valid obstacles in the array.
 * @param current_pos       Current position of the drone [x, y, z].
 * @param current_rpy       Current roll, pitch, yaw of the drone [r, p, y].
 * @param search_radius     Radius around current_pos to consider for analysis.
 * @param result            Pointer to psmsl_depth_result_t to store the analysis output.
 */
void psmsl_depth_analyze_obstacles(const float obstacles[][4], int num_obstacles,
                                   const float current_pos[3], const float current_rpy[3],
                                   float search_radius,
                                   psmsl_depth_result_t *result);

/**
 * @brief Processes a raw normalized depth image directly on the MCU.
 *
 * @param depth_image       Pointer to a flat array of depth values [0.0, 1.0].
 * @param width             Width of the depth image (e.g., 128).
 * @param height            Height of the depth image (e.g., 128).
 * @param current_pos       Current position of the drone [x, y, z].
 * @param current_rpy       Current roll, pitch, yaw of the drone [r, p, y].
 * @param max_range         Maximum depth range in meters.
 * @param fov_deg           Camera field of view in degrees.
 * @param search_radius     Radius around current_pos to consider for analysis.
 * @param result            Pointer to psmsl_depth_result_t to store the analysis output.
 */
void psmsl_depth_analyze_image(const float *depth_image, int width, int height,
                               const float current_pos[3], const float current_rpy[3],
                               float max_range, float fov_deg, float search_radius,
                               psmsl_depth_result_t *result);

#ifdef __cplusplus
}
#endif

#endif // PSMSL_DEPTH_PROCESSOR_H
