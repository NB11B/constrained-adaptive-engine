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
    float nav_impact = (1.0f - ctrl->state.navigability_score) * 3.5f;
    float target_breathing = fminf(3.5f, 1.0f + clutter_impact + nav_impact);
    
    // Temporal Smoothing: "Expand and Contract" over time
    float alpha = 0.05f; // Smoothing coefficient (approx. 1s time constant at 50Hz)
    ctrl->state.temporal_breathing = (1.0f - alpha) * ctrl->state.temporal_breathing + alpha * target_breathing;
    // Enforce a breathing floor to prevent the engine from freezing entirely near the ground
    float breathing_factor = fmaxf(0.001f, ctrl->state.temporal_breathing);
    if (ctrl->state.landing_phase) {
        breathing_factor = 0.8f;
    }
    
    // Adaptive gains: safety prioritized in high-clutter
    float k_att = ctrl->params.attraction_gain / (breathing_factor * 0.85f);
    float k_repu_base = ctrl->params.mode_v * breathing_factor * 1.50f;

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
            // Proactive boost for early clearing (starts when slightly below the top).
            float vertical_boost = (dz > -0.2f) ? 5.5f : 1.0f;
            
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

    // --- Local Minimum Detector & Escape Strategy ---
    float current_z = ctrl->state.current_pos[2];
    float tgt_z = ctrl->state.target_pos[2];
    float z_dist_lock = current_z - tgt_z;
    float dist_xy_lock = sqrtf((ctrl->state.current_pos[0] - ctrl->state.target_pos[0]) * (ctrl->state.current_pos[0] - ctrl->state.target_pos[0]) +
                               (ctrl->state.current_pos[1] - ctrl->state.target_pos[1]) * (ctrl->state.current_pos[1] - ctrl->state.target_pos[1]) + 1e-8f);

    float speed_current = sqrtf(ctrl->state.current_vel[0]*ctrl->state.current_vel[0] + 
                                ctrl->state.current_vel[1]*ctrl->state.current_vel[1] + 
                                ctrl->state.current_vel[2]*ctrl->state.current_vel[2]);

    // ── LAYER 1: Predictive Detector ─────────────────────────────────────────
    // Fires ~0.4s before a stall by blending PSMSL threat signals.
    // Disabled during landing/descent to avoid aborting a precision approach.
    bool in_approach = ctrl->state.landing_phase || ctrl->state.descent_phase;
    if (!in_approach) {
        // Blended threat scalar: collision_risk dominates, clutter and nav supplement
        float raw_threat = (ctrl->state.collision_risk_score * 0.50f) +
                           (ctrl->state.clutter_density      * 0.30f) +
                           ((1.0f - ctrl->state.navigability_score) * 0.20f);
        // Fast-attack EMA (alpha=0.3 → ~6-tick time constant at 50Hz)
        ctrl->state.predictive_threat_score = 0.70f * ctrl->state.predictive_threat_score + 0.30f * raw_threat;

        // Leaky integrator: accumulates under sustained threat + slow speed
        if (ctrl->state.predictive_threat_score > 0.55f && speed_current < 0.8f) {
            ctrl->state.threat_accumulator += 1.0f;
        } else if (speed_current > 1.2f) {
            // Clear progress resets the threat at 2x the accumulation rate
            ctrl->state.threat_accumulator -= 2.0f;
        } else {
            ctrl->state.threat_accumulator -= 0.5f;
        }
        if (ctrl->state.threat_accumulator < 0.0f) ctrl->state.threat_accumulator = 0.0f;

        // Fire predictive escape at threshold (approx 0.4s of degraded conditions)
        if (ctrl->state.threat_accumulator > 20.0f && !ctrl->state.predictive_escape_active
            && !ctrl->state.escape_mode_active) {
            ctrl->state.predictive_escape_active = true;
            ctrl->state.predictive_escape_ticks = 0;
            // Climb above the obstacle field: higher in clutter, lower in open space
            ctrl->state.predictive_escape_target_z = current_z
                + 1.0f + ctrl->state.clutter_density * 2.0f;
            ctrl->state.threat_accumulator = 0.0f;
        }
    }

    // Execute predictive escape: climb to clear altitude, then release
    if (ctrl->state.predictive_escape_active) {
        ctrl->state.predictive_escape_ticks++;
        float climb_err = ctrl->state.predictive_escape_target_z - current_z;
        // Bypass vz smoother on first tick for immediate response
        if (ctrl->state.predictive_escape_ticks == 1) {
            desired_vz = fminf(2.0f, fmaxf(0.5f, climb_err * 2.0f));
        } else {
            desired_vz = fminf(1.5f, fmaxf(0.0f, climb_err * 1.5f));
        }
        // Release when altitude reached or after 5s timeout (250 ticks)
        if (climb_err < 0.15f || ctrl->state.predictive_escape_ticks > 250) {
            ctrl->state.predictive_escape_active = false;
            ctrl->state.predictive_escape_ticks = 0;
        }
    }

    // ── LAYER 2: Reactive Escape (safety net, fires after 1.5s of zero motion) ─
    if (speed_current < 0.15f && dist_xy_lock > 0.15f) {
        ctrl->state.stuck_counter++;
    } else {
        if (ctrl->state.stuck_counter > 0) ctrl->state.stuck_counter--;
    }

    if (ctrl->state.stuck_counter > 75 && !ctrl->state.escape_mode_active
        && !ctrl->state.predictive_escape_active) {
        ctrl->state.escape_mode_active = true;
        float grad_norm = sqrtf(gradient[0]*gradient[0] + gradient[1]*gradient[1] + 1e-8f);
        if (grad_norm > 0.1f) {
            ctrl->state.escape_vector[0] = -gradient[1] / grad_norm;
            ctrl->state.escape_vector[1] =  gradient[0] / grad_norm;
            ctrl->state.escape_vector[2] = 1.5f;
        } else {
            ctrl->state.escape_vector[0] = 1.0f;
            ctrl->state.escape_vector[1] = 1.0f;
            ctrl->state.escape_vector[2] = 1.5f;
        }
    }

    if (ctrl->state.escape_mode_active) {
        if (speed_current > 1.0f || ctrl->state.stuck_counter > 150) {
            ctrl->state.escape_mode_active = false;
            ctrl->state.stuck_counter = 0;
        } else {
            desired_vx = ctrl->state.escape_vector[0] * 2.0f;
            desired_vy = ctrl->state.escape_vector[1] * 2.0f;
            desired_vz = ctrl->state.escape_vector[2];
        }
    }
    // ─────────────────────────────────────────────────────────────────────────

    // Z-attraction (proportional control to cruise altitude, or target Z during landing)
    float target_z_ref = ctrl->params.cruise_altitude;
    if (ctrl->state.landing_phase) {
        float hover_offset = 0.3f + 0.9f * jnp_clip_placeholder(dist_xy_lock / 1.5f);
        target_z_ref = tgt_z + hover_offset;
    }
    float dz_attraction = target_z_ref - current_z;
    desired_vz += ctrl->params.attraction_gain * 2.0f * dz_attraction;

    if (ctrl->state.landing_phase) {
        if (dist_xy_lock > 4.5f && !ctrl->state.descent_phase) {
            ctrl->state.landing_phase = false;
            ctrl->state.descent_phase = false;
        }
    } else {
        // Enter landing_phase only when close laterally AND above the pad.
        // This prevents the drone from entering landing mode while still at ground level.
        if (dist_xy_lock < 2.5f && current_z > tgt_z + 0.5f) ctrl->state.landing_phase = true;
    }

    if (ctrl->state.landing_phase && !ctrl->state.descent_phase) {
        float cur_vx_rel = ctrl->state.current_vel[0] - ctrl->params.platform_vel_est[0];
        float cur_vy_rel = ctrl->state.current_vel[1] - ctrl->params.platform_vel_est[1];
        float vxy_rel = sqrtf(cur_vx_rel * cur_vx_rel + cur_vy_rel * cur_vy_rel + 1e-8f);
        float adaptive_vxy_gate = fminf(2.5f, fmaxf(1.0f, 1.0f + 1.5f * (z_dist_lock / 2.0f)));

        // Altimeter void guard: If we are physically low but AGL reads high, trust physical Z
        float effective_z_dist = z_dist_lock;
        if (ctrl->state.agl > 15.0f && current_z < 0.5f) {
            effective_z_dist = current_z; // Trust the absolute Z when altimeter voids
        }

        // Descent phase gate (three conditions must all hold):
        //   1. XY within 1.5x threshold (0.60 * 1.5 = 0.90m) — close enough laterally
        //   2. Must be ABOVE the pad by at least 0.30m — prevents lateral ground-level approach
        //   3. Z distance within 3.0x threshold — altitude gate
        //   4. Lateral speed within adaptive gate
        bool xy_ok  = dist_xy_lock < (ctrl->params.landing_threshold_xy * 1.5f);
        // alt_ok: must be above pad (>0.30m) but no upper limit — drone may approach from high altitude
        bool alt_ok = effective_z_dist > 0.30f;
        bool vxy_ok = vxy_rel < adaptive_vxy_gate;
        if (xy_ok && alt_ok && vxy_ok) {
            ctrl->state.descent_phase = true;
            ctrl->state.descent_vz = ctrl->state.current_vel[2];
        }
    }

    float max_total_speed = ctrl->params.max_speed;

    if (ctrl->state.descent_phase) {
        float h_rem = fmaxf(current_z - tgt_z, 0.0f);

        // --- Stepped Feathering Profile (Landing Fix) ---
        // Three-stage descent: approach (-0.8), feather (-0.3), press (-0.25).
        // Floor guard at tgt_z+0.01 stops pressing once contact is made.
        float vz_feather;
        if (current_z < tgt_z + 0.01f) {
            // Floor guard: we are at or below pad surface — stop pressing
            vz_feather = 0.0f;
        } else if (h_rem > 0.50f) {
            // Approach: descend at -0.8 m/s until 0.5m above pad
            vz_feather = -0.80f;
        } else if (h_rem > 0.10f) {
            // Feather: slow to -0.3 m/s between 0.5m and 0.1m
            vz_feather = -0.30f;
        } else {
            // Final press: gentle -0.25 m/s for last 0.1m
            vz_feather = -0.25f;
        }

        // Add platform vertical velocity compensation
        float vz_target = vz_feather + ctrl->params.platform_vel_est[2];
        desired_vz = fminf(0.5f, fmaxf(-1.0f, (vz_target - ctrl->state.current_vel[2]) * 1.5f + vz_target));

        // --- Precise Station-Keeping ---
        // Tighten horizontal control as we get closer to the pad
        float speed_xy = fminf(0.5f, fmaxf(0.05f, dist_xy_lock * 1.5f));
        float grad_norm = sqrtf(desired_vx * desired_vx + desired_vy * desired_vy + 1e-8f);
        
        // Stronger centering force during final descent
        float centering_bias = (h_rem < 0.3f) ? 1.2f : 1.0f;
        
        // Kill-zone commitment: Once within 1.0m XY, ignore the potential field gradient
        // entirely and use pure centering toward the pad. This prevents obstacle repulsion
        // from kicking the drone sideways during the final descent.
        if (dist_xy_lock < 1.0f) {
            // Pure centering: move directly toward pad center, ignore gradient
            float dx_to_pad = ctrl->state.target_pos[0] - ctrl->state.current_pos[0];
            float dy_to_pad = ctrl->state.target_pos[1] - ctrl->state.current_pos[1];
            float d_to_pad = sqrtf(dx_to_pad * dx_to_pad + dy_to_pad * dy_to_pad + 1e-8f);
            if (dist_xy_lock < 0.15f) {
                // Directly above pad: pure vertical drop
                desired_vx = ctrl->params.platform_vel_est[0];
                desired_vy = ctrl->params.platform_vel_est[1];
            } else {
                desired_vx = (dx_to_pad / d_to_pad) * speed_xy + ctrl->params.platform_vel_est[0];
                desired_vy = (dy_to_pad / d_to_pad) * speed_xy + ctrl->params.platform_vel_est[1];
            }
        } else {
            desired_vx = (desired_vx / grad_norm) * speed_xy * centering_bias + ctrl->params.platform_vel_est[0] * 1.00f;
            desired_vy = (desired_vy / grad_norm) * speed_xy * centering_bias + ctrl->params.platform_vel_est[1] * 1.00f;
        }
        
        max_total_speed = 1.0f;
    } else if (ctrl->state.landing_phase) {
        float speed_xy = fminf(1.0f, fmaxf(0.08f, dist_xy_lock * 1.2f));
        float grad_norm = sqrtf(desired_vx * desired_vx + desired_vy * desired_vy + 1e-8f);
        desired_vx = (desired_vx / grad_norm) * speed_xy + ctrl->params.platform_vel_est[0] * 0.90f;
        desired_vy = (desired_vy / grad_norm) * speed_xy + ctrl->params.platform_vel_est[1] * 0.90f;
        max_total_speed = 1.5f;
        // If below hover target, force upward — do not allow downward drift
        if (current_z < target_z_ref - 0.10f) {
            desired_vz = fminf(1.2f, fmaxf(0.30f, desired_vz));
        } else {
            desired_vz = fminf(0.4f, fmaxf(-0.4f, desired_vz));
        }
    } else {
        float speed_xy = 2.2f; // Matches JAX reference
        if (dist_xy_lock < 20.0f) speed_xy = 2.0f + fminf(1.0f, fmaxf(0.0f, (dist_xy_lock - 5.0f) / 15.0f)) * 0.5f;
        float grad_norm = sqrtf(desired_vx * desired_vx + desired_vy * desired_vy + 1e-8f);
        desired_vx = (desired_vx / grad_norm) * speed_xy + ctrl->params.platform_vel_est[0] * 0.85f;
        desired_vy = (desired_vy / grad_norm) * speed_xy + ctrl->params.platform_vel_est[1] * 0.85f;
        max_total_speed = 3.0f; // Matches physical max speed
        desired_vz = fminf(1.8f, fmaxf(-1.6f, desired_vz));
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

    float max_dv = 2.0f * SIM_DT; // Matches JAX reference max_accel = 2.0 m/s2
    float dv_x = desired_vx - ctrl->state.last_vel_cmd[0];
    float dv_y = desired_vy - ctrl->state.last_vel_cmd[1];
    float dv_z = desired_vz - ctrl->state.last_vel_cmd[2];
    float dv_norm = sqrtf(dv_x * dv_x + dv_y * dv_y + dv_z * dv_z + 1e-8f);

    // Emergency climb: if the drone is dangerously low (below cruise_altitude - 1m) and
    // needs to climb, allow faster vz correction (8x normal) to arrest descent.
    // This applies in both cruise and landing_phase (but not descent_phase).
    float max_dv_z = max_dv;
    if (!ctrl->state.descent_phase) {
        float danger_floor = ctrl->params.cruise_altitude - 1.0f;
        // Also trigger if below landing hover target in landing_phase
        if (ctrl->state.landing_phase) {
            float hover_target = tgt_z + 0.3f + 0.9f * fminf(1.0f, fmaxf(0.0f, dist_xy_lock / 1.5f));
            danger_floor = fminf(danger_floor, hover_target - 0.10f);
        }
        if (current_z < danger_floor && desired_vz > ctrl->state.last_vel_cmd[2]) {
            max_dv_z = max_dv * 8.0f; // Emergency climb: 8x acceleration
        }
        // Ground contact escape: if drone is at ground level and not in descent,
        // override the smoother entirely and force a strong upward command.
        if (current_z < 0.15f) {
            desired_vz = 1.8f; // Full upward command
            ctrl->state.last_vel_cmd[2] = 1.8f; // Bypass smoother
        }
    }

    if (dv_norm > max_dv) {
        dv_x = (dv_x / dv_norm) * max_dv;
        dv_y = (dv_y / dv_norm) * max_dv;
        // Apply separate (potentially higher) limit for vz
        float dv_z_limited = fminf(fabsf(dv_z), max_dv_z) * (dv_z >= 0.0f ? 1.0f : -1.0f);
        dv_z = dv_z_limited;
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
    controller->state.temporal_breathing = 1.0f;
    controller->state.convergence = 0.0f;
    controller->state.converged = false;
    controller->state.iterations = 0;
    controller->state.landing_phase = false;
    controller->state.descent_phase = false;
    // Reset predictive detector state
    controller->state.predictive_threat_score = 0.0f;
    controller->state.threat_accumulator = 0.0f;
    controller->state.predictive_escape_active = false;
    controller->state.predictive_escape_target_z = 0.0f;
    controller->state.predictive_escape_ticks = 0;
    // Reset reactive escape state
    controller->state.stuck_counter = 0;
    controller->state.escape_mode_active = false;
    memset(controller->state.escape_vector, 0, sizeof(float) * 3);
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

void adapt_process_sensor_data(adaptation_controller_t *controller,
                               const float current_pos[3],
                               const float current_vel[3],
                               const float current_rpy[3],
                               const float target_pos[3],
                               float yaw_rate,
                               float agl,
                               const float *depth_image,
                               int depth_width,
                               int depth_height,
                               float max_range,
                               float fov_deg)
{
    if (!controller) return;
    memcpy(controller->state.current_pos, current_pos, sizeof(float) * 3);
    memcpy(controller->state.current_vel, current_vel, sizeof(float) * 3);
    memcpy(controller->state.current_rpy, current_rpy, sizeof(float) * 3);
    memcpy(controller->state.target_pos, target_pos, sizeof(float) * 3);
    controller->state.yaw_rate = yaw_rate;
    controller->state.agl = agl;
    
    // We no longer copy obstacles directly. The depth processor extracts them internally.
    controller->num_internal_obstacles = 0; // Clear old obstacles

    psmsl_depth_result_t depth_analysis_result;
    if (depth_image && depth_width > 0 && depth_height > 0) {
        // Search radius must cover the full sensor range, not just the safety bubble.
        // Use max_range * 0.8 to catch all relevant obstacles while ignoring far background.
        float depth_search_radius = max_range * 0.8f;
        psmsl_depth_analyze_image(depth_image, depth_width, depth_height,
                                  controller->state.current_pos, controller->state.current_rpy,
                                  max_range, fov_deg, depth_search_radius,
                                  controller->internal_obstacles, &controller->num_internal_obstacles, ADAPT_MAX_OBSTACLES,
                                  &depth_analysis_result);
    } else {
        depth_analysis_result.clutter_density = 0.0f;
        depth_analysis_result.local_navigability = 1.0f;
        depth_analysis_result.collision_risk_score = 0.0f;
    }
    controller->state.clutter_density = depth_analysis_result.clutter_density;
    controller->state.navigability_score = depth_analysis_result.local_navigability;
    controller->state.collision_risk_score = depth_analysis_result.collision_risk_score;
}

const adapt_state_t* adapt_get_state(const adaptation_controller_t *controller) { return controller ? &controller->state : NULL; }

int adapt_export_state_json(const adaptation_controller_t *controller, char *buffer, size_t buffer_size)
{
    if (!controller || !buffer || buffer_size == 0) return 0;
    return snprintf(buffer, buffer_size, "{\"mode\":%d,\"current_pos\":[%.2f,%.2f,%.2f],\"collision_risk\":%.2f}", controller->state.mode, controller->state.current_pos[0], controller->state.current_pos[1], controller->state.current_pos[2], controller->state.collision_risk_score);
}
