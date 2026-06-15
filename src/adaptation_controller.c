/**
 * @file adaptation_controller.c
 * @brief High-Performance MCU-Compliant Analytical Autopilot for Swarm Subnet 124
 */

#include "adaptation_controller.h"
#include "psmsl_depth_processor.h"

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Mock function to replace JAX's jnp.clip functionality in C
float jnp_clip_placeholder(float val) {
    if (val < 0.0f) return 0.0f;
    if (val > 1.0f) return 1.0f;
    return val;
}

// ── Low-Level Physics and Boundary Thresholds ──────────────────────────────
#define SIM_DT                      0.02f   // 50 Hz control loop clock
#define MAX_SPEED_PHYSICAL          3.0f    // m/s physical absolute ceiling
#define CONVERGENCE_THRESHOLD_POS   0.10f   // meters
#define CONVERGENCE_THRESHOLD_VEL   0.05f   // m/s

// =============================================================================
// Private Mathematical and Navigational Subroutines
// =============================================================================

static void calculate_potential_gradient(adaptation_controller_t *ctrl,
                                         const float current_pos[3],
                                         const float target_pos[3],
                                         const float obstacles[][4], int num_obstacles,
                                         float gradient[3])
{
    gradient[0] = 0.0f;
    gradient[1] = 0.0f;
    gradient[2] = 0.0f;

    // --- Dynamic Adaptation via PSMSL Metrics (Breathing Profiles) ---
    // The "breathing" factor expands in clutter and contracts in open space.
    // We incorporate navigability_score (lower is worse) to expand margins in complex geometry.
    float clutter_impact = ctrl->state.clutter_density * 5.0f;
    float nav_impact = (1.0f - ctrl->state.local_navigability) * 2.0f;
    float breathing_factor = fminf(3.0f, 1.0f + clutter_impact + nav_impact);
    
    // Adaptive gains: safety prioritized in high-clutter
    float k_att = ctrl->params.attraction_gain / breathing_factor;
    float k_repu_base = ctrl->params.mode_v * breathing_factor;

    float dx_tgt = current_pos[0] - target_pos[0];
    float dy_tgt = current_pos[1] - target_pos[1];
    float att_norm = sqrtf(dx_tgt * dx_tgt + dy_tgt * dy_tgt + 1e-8f);
    
    // Attraction force (normalized)
    gradient[0] += k_att * (dx_tgt / att_norm) * 4.0f;
    gradient[1] += k_att * (dy_tgt / att_norm) * 4.0f;

    // Velocity-aware safety: buffer expands based on current speed
    float speed_current = sqrtf(ctrl->state.current_vel[0]*ctrl->state.current_vel[0] + 
                                ctrl->state.current_vel[1]*ctrl->state.current_vel[1] + 
                                ctrl->state.current_vel[2]*ctrl->state.current_vel[2] + 1e-8f);
    float dynamic_safety_buffer = ctrl->params.safety_radius * (1.0f + 0.25f * speed_current);

    for (int i = 0; i < num_obstacles; i++) {
        float ox = obstacles[i][0];
        float oy = obstacles[i][1];
        float oz = obstacles[i][2];
        float or = obstacles[i][3];

        if (oz >= 990.0f || (ox == 0.0f && oy == 0.0f && oz == 0.0f)) continue;

        // Don't repel from obstacles that are extremely close to the target pad
        float obs_to_tgt_xy = sqrtf((ox - target_pos[0]) * (ox - target_pos[0]) +
                                    (oy - target_pos[1]) * (oy - target_pos[1]) + 1e-8f);
        if (obs_to_tgt_xy < 1.1f) continue;

        float dx = current_pos[0] - ox;
        float dy = current_pos[1] - oy;
        float dz = current_pos[2] - oz;
        float dist = sqrtf(dx * dx + dy * dy + dz * dz + 1e-8f);
        
        // Effective safety radius "breathes" based on environmental feedback
        float effective_safety_radius = (dynamic_safety_buffer + or) * breathing_factor;

        if (dist < effective_safety_radius && dist > 0.01f) {
            float inv_r = 1.0f / effective_safety_radius;
            float inv_d = 1.0f / dist;
            
            // --- 3D Aware Repulsion ---
            // If we are above the obstacle, we primarily want to push UP to clear it.
            float vertical_boost = (dz > 0.0f) ? 2.5f : 1.0f;
            
            float scalar = k_repu_base * (inv_r - inv_d) * (inv_d * inv_d * inv_d);
            
            const float MAX_REPU_SCALAR = 25000.0f;
            if (scalar < -MAX_REPU_SCALAR) scalar = -MAX_REPU_SCALAR;
            
            gradient[0] += scalar * dx;
            gradient[1] += scalar * dy;
            gradient[2] += scalar * dz * vertical_boost;
        }
    }
}

static void calculate_control_output(adaptation_controller_t *ctrl,
                                     const float gradient[3],
                                     float control_output[5])
{
    float desired_vx = -gradient[0];
    float desired_vy = -gradient[1];
    float desired_vz = -gradient[2];

    float current_z = ctrl->state.current_pos[2];
    float tgt_z = ctrl->state.target_pos[2];
    float z_dist_lock = current_z - tgt_z;
    float dist_xy_lock = sqrtf((ctrl->state.current_pos[0] - ctrl->state.target_pos[0]) * (ctrl->state.current_pos[0] - ctrl->state.target_pos[0]) +
                               (ctrl->state.current_pos[1] - ctrl->state.target_pos[1]) * (ctrl->state.current_pos[1] - ctrl->state.target_pos[1]) + 1e-8f);

    // Z-attraction (proportional control to cruise altitude, or target Z during landing)
    float target_z_ref = tgt_z + 2.5f; // Default cruise altitude relative to pad
    if (ctrl->state.landing_phase) {
        float hover_offset = 0.1f + 0.6f * jnp_clip_placeholder(dist_xy_lock / 2.0f);
        target_z_ref = tgt_z + hover_offset;
    }
    float dz_attraction = target_z_ref - current_z;
    desired_vz += ctrl->params.attraction_gain * 1.5f * dz_attraction;

    if (ctrl->state.landing_phase) {
        if (dist_xy_lock > 3.0f && !ctrl->state.descent_phase) {
            ctrl->state.landing_phase = false;
            ctrl->state.descent_phase = false;
        }
    } else {
        if (dist_xy_lock < 6.0f) ctrl->state.landing_phase = true;
    }

    if (ctrl->state.landing_phase && !ctrl->state.descent_phase) {
        float cur_vx_rel = ctrl->state.current_vel[0] - ctrl->params.platform_vel_est[0];
        float cur_vy_rel = ctrl->state.current_vel[1] - ctrl->params.platform_vel_est[1];
        float vxy_rel = sqrtf(cur_vx_rel * cur_vx_rel + cur_vy_rel * cur_vy_rel + 1e-8f);
        float adaptive_vxy_gate = fminf(1.5f, fmaxf(0.5f, 0.5f + 1.0f * (z_dist_lock / 1.5f)));

        if (dist_xy_lock < (ctrl->params.landing_threshold_xy * 1.5f) && z_dist_lock < (ctrl->params.landing_threshold_z * 1.5f) && z_dist_lock > -0.1f && vxy_rel < adaptive_vxy_gate) {
            ctrl->state.descent_phase = true;
            ctrl->state.descent_vz = ctrl->state.current_vel[2];
        }
    }

    float max_total_speed = ctrl->params.max_speed;

    if (ctrl->state.descent_phase) {
        float h_rem = fmaxf(current_z - tgt_z, 0.0f);
        
        // --- Dynamic Feathering (Flare) Profile ---
        // We use a feathered vertical velocity that decays as we approach touchdown
        float vz_feather;
        if (h_rem < 0.05f) {
            // Final touchdown latch
            vz_feather = -ctrl->params.landing_descent_rate * 4.0f; 
        } else {
            // Exponential flare: faster at top, slowing down as we get closer
            float flare_base = 0.65f;
            vz_feather = -fmaxf(0.08f, flare_base * powf(h_rem / 1.5f, 0.75f));
        }
        
        // Add platform vertical velocity compensation
        float vz_target = vz_feather + ctrl->params.platform_vel_est[2];
        desired_vz = fminf(0.5f, fmaxf(-1.0f, (vz_target - ctrl->state.current_vel[2]) * 1.5f + vz_target));

        // --- Precise Station-Keeping ---
        // Tighten horizontal control as we get closer to the pad
        float speed_xy = fminf(0.6f, fmaxf(0.02f, dist_xy_lock * 2.5f));
        float grad_norm = sqrtf(desired_vx * desired_vx + desired_vy * desired_vy + 1e-8f);
        
        // Stronger centering force during final descent
        float centering_bias = (h_rem < 0.3f) ? 1.2f : 1.0f;
        desired_vx = (desired_vx / grad_norm) * speed_xy * centering_bias + ctrl->params.platform_vel_est[0] * 0.95f;
        desired_vy = (desired_vy / grad_norm) * speed_xy * centering_bias + ctrl->params.platform_vel_est[1] * 0.95f;
        
        max_total_speed = 1.5f;
    } else if (ctrl->state.landing_phase) {
        float speed_xy = fminf(2.0f, fmaxf(0.15f, dist_xy_lock * 1.5f));
        float grad_norm = sqrtf(desired_vx * desired_vx + desired_vy * desired_vy + 1e-8f);
        desired_vx = (desired_vx / grad_norm) * speed_xy + ctrl->params.platform_vel_est[0] * 0.90f;
        desired_vy = (desired_vy / grad_norm) * speed_xy + ctrl->params.platform_vel_est[1] * 0.90f;
        max_total_speed = 2.0f;
    } else {
        float speed_xy = 5.5f;
        if (dist_xy_lock < 15.0f) speed_xy = 3.5f + fminf(1.0f, fmaxf(0.0f, (dist_xy_lock - 5.0f) / 10.0f)) * 3.5f;
        float grad_norm = sqrtf(desired_vx * desired_vx + desired_vy * desired_vy + 1e-8f);
        desired_vx = (desired_vx / grad_norm) * speed_xy + ctrl->params.platform_vel_est[0] * 0.90f;
        desired_vy = (desired_vy / grad_norm) * speed_xy + ctrl->params.platform_vel_est[1] * 0.90f;
        max_total_speed = 6.0f;
    }

    if (ctrl->state.landing_phase) {
        float damping_factor = fminf(0.45f, fmaxf(0.0f, 0.45f * (1.0f - dist_xy_lock / 2.5f)));
        desired_vx -= damping_factor * (ctrl->state.current_vel[0] - ctrl->params.platform_vel_est[0]);
        desired_vy -= damping_factor * (ctrl->state.current_vel[1] - ctrl->params.platform_vel_est[1]);
    }

    float total_speed = sqrtf(desired_vx * desired_vx + desired_vy * desired_vy + desired_vz * desired_vz + 1e-8f);
    if (total_speed > max_total_speed) {
        desired_vx = (desired_vx / total_speed) * max_total_speed;
        desired_vy = (desired_vy / total_speed) * max_total_speed;
        desired_vz = (desired_vz / total_speed) * max_total_speed;
    }

    float max_dv = 8.0f * SIM_DT;
    float dv_x = desired_vx - ctrl->state.last_vel_cmd[0];
    float dv_y = desired_vy - ctrl->state.last_vel_cmd[1];
    float dv_z = desired_vz - ctrl->state.last_vel_cmd[2];
    float dv_norm = sqrtf(dv_x * dv_x + dv_y * dv_y + dv_z * dv_z + 1e-8f);

    if (dv_norm > max_dv) {
        dv_x = (dv_x / dv_norm) * max_dv;
        dv_y = (dv_y / dv_norm) * max_dv;
        dv_z = (dv_z / dv_norm) * max_dv;
    }

    ctrl->state.control_output[0] = ctrl->state.last_vel_cmd[0] + dv_x;
    ctrl->state.control_output[1] = ctrl->state.last_vel_cmd[1] + dv_y;
    ctrl->state.control_output[2] = ctrl->state.last_vel_cmd[2] + dv_z;
    memcpy(ctrl->state.last_vel_cmd, ctrl->state.control_output, sizeof(float) * 3);
    ctrl->state.control_output[3] = sqrtf(ctrl->state.control_output[0]*ctrl->state.control_output[0] + ctrl->state.control_output[1]*ctrl->state.control_output[1] + ctrl->state.control_output[2]*ctrl->state.control_output[2]);

    float desired_yaw = ctrl->state.current_rpy[2];
    if (dist_xy_lock > 0.15f) desired_yaw = atan2f(ctrl->state.target_pos[1] - ctrl->state.current_pos[1], ctrl->state.target_pos[0] - ctrl->state.current_pos[0]);
    float yaw_error_wrapped = atan2f(sinf(desired_yaw - ctrl->state.current_rpy[2]), cosf(desired_yaw - ctrl->state.current_rpy[2]));
    ctrl->state.control_output[4] = fminf(1.0f, fmaxf(-1.0f, (ctrl->state.current_rpy[2] + yaw_error_wrapped - (0.15f * ctrl->state.yaw_rate)) / (float)M_PI));
}

adaptation_controller_t* adapt_init()
{
    adaptation_controller_t *controller = (adaptation_controller_t*)malloc(sizeof(adaptation_controller_t));
    if (!controller) return NULL;
    memset(controller, 0, sizeof(adaptation_controller_t));

    // Set default parameters (tuned from successful flights)
    controller->params.cruise_altitude = 1.5f; // meters
    controller->params.safety_radius = 1.15f;   // meters (base)
    controller->params.mode_v = 200.0f;          // Repulsion strength (base)
    controller->params.attraction_gain = 8.5f; // Attraction strength (base)
    controller->params.max_speed = 5.0f;       // m/s
    controller->params.max_yaw_rate = 1.5f;    // rad/s
    controller->params.landing_descent_rate = 0.025f; // m/s
    controller->params.landing_threshold_xy = 0.60f; // meters
    controller->params.landing_threshold_z = 1.20f; // meters
    memset(controller->params.platform_vel_est, 0, sizeof(controller->params.platform_vel_est));

    controller->state.mode = ADAPT_MODE_OFF;
    controller->state.speed = ADAPT_SPEED_MEDIUM;
    controller->initialized = true;
    return controller;
}

void adapt_free(adaptation_controller_t *controller) { if (controller) free(controller); }

void adapt_reset(adaptation_controller_t *controller)
{
    if (!controller) return;
    memset(controller->state.current_pos, 0, sizeof(float) * 3);
    memset(controller->state.current_vel, 0, sizeof(float) * 3);
    memset(controller->state.current_rpy, 0, sizeof(float) * 3);
    memset(controller->state.target_pos, 0, sizeof(float) * 3);
    memset(controller->state.control_output, 0, sizeof(float) * 5);
    memset(controller->state.last_vel_cmd, 0, sizeof(float) * 3);
    controller->state.collision_detected = false;
    controller->state.target_reached = false;
    controller->state.convergence = 0.0f;
    controller->state.converged = false;
    controller->state.iterations = 0;
    controller->state.landing_phase = false;
    controller->state.descent_phase = false;
}

void adapt_set_flight_params(adaptation_controller_t *controller, const adapt_flight_params_t *params)
{
    if (!controller || !params) return;
    memcpy(&controller->params, params, sizeof(adapt_flight_params_t));
}

void adapt_set_mode(adaptation_controller_t *controller, adapt_mode_t mode) { if (controller) controller->state.mode = mode; }
void adapt_set_speed(adaptation_controller_t *controller, adapt_speed_t speed) { if (controller) controller->state.speed = speed; }
void adapt_set_target_pos(adaptation_controller_t *controller, const float target_pos[3]) { if (controller && target_pos) memcpy(controller->state.target_pos, target_pos, sizeof(float) * 3); }

bool adapt_start_adaptive(adaptation_controller_t *controller)
{
    if (!controller || !controller->initialized) return false;
    controller->state.mode = ADAPT_MODE_ADAPTIVE;
    controller->running = true;
    return true;
}

void adapt_stop(adaptation_controller_t *controller) { if (controller) { controller->state.mode = ADAPT_MODE_HOLD; controller->running = false; } }

bool adapt_update(adaptation_controller_t *controller)
{
    if (!controller || !controller->running) return false;
    
    // Calculate potential field gradient (now using updated PSMSL state from process_sensor_data)
    float gradient[3];
    calculate_potential_gradient(controller, controller->state.current_pos, controller->state.target_pos, controller->internal_obstacles, controller->num_internal_obstacles, gradient);
    
    // Process control loop
    calculate_control_output(controller, gradient, controller->state.control_output);
    
    controller->state.iterations++;
    return true;
}

void adapt_process_sensor_data(adaptation_controller_t *controller, const float current_pos[3], const float current_vel[3], const float current_rpy[3], const float target_pos[3], float yaw_rate, float agl, const float obstacles[][4], int num_obstacles)
{
    if (!controller) return;
    memcpy(controller->state.current_pos, current_pos, sizeof(float) * 3);
    memcpy(controller->state.current_vel, current_vel, sizeof(float) * 3);
    memcpy(controller->state.current_rpy, current_rpy, sizeof(float) * 3);
    // Only update target if not in landing/descent phases or if explicitly requested
    if (!controller->state.landing_phase && !controller->state.descent_phase) {
        memcpy(controller->state.target_pos, target_pos, sizeof(float) * 3);
    }
    controller->state.yaw_rate = yaw_rate;
    controller->state.agl = agl;
    if (obstacles && num_obstacles > 0) {
        int to_copy = num_obstacles > 256 ? 256 : num_obstacles;
        memcpy(controller->internal_obstacles, obstacles, sizeof(float) * 4 * to_copy);
        controller->num_internal_obstacles = to_copy;
    }
    psmsl_depth_result_t depth_analysis_result;
    psmsl_depth_analyze_obstacles(controller->internal_obstacles, controller->num_internal_obstacles, controller->state.current_pos, controller->state.current_rpy, controller->params.safety_radius * 2.0f, &depth_analysis_result);
    controller->state.clutter_density = depth_analysis_result.clutter_density;
    controller->state.local_navigability = depth_analysis_result.local_navigability;
    controller->state.collision_risk_score = depth_analysis_result.collision_risk_score;
}

const adapt_state_t* adapt_get_state(const adaptation_controller_t *controller) { return controller ? &controller->state : NULL; }

int adapt_export_state_json(const adaptation_controller_t *controller, char *buffer, size_t buffer_size)
{
    if (!controller || !buffer || buffer_size == 0) return 0;
    return snprintf(buffer, buffer_size, "{\"mode\":%d,\"current_pos\":[%.2f,%.2f,%.2f],\"collision_risk\":%.2f}", controller->state.mode, controller->state.current_pos[0], controller->state.current_pos[1], controller->state.current_pos[2], controller->state.collision_risk_score);
}
