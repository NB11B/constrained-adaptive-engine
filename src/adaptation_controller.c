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

static inline float clampf_local(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static inline float norm2f(float x, float y) {
    return sqrtf(x * x + y * y + 1e-8f);
}

static inline float norm3f(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z + 1e-8f);
}

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

    float clutter_impact = ctrl->state.clutter_density * 4.0f;
    float nav_impact = (1.0f - ctrl->state.navigability_score) * 2.5f;
    float target_breathing = fminf(3.0f, 1.0f + clutter_impact + nav_impact);
    float alpha = 0.05f;
    ctrl->state.temporal_breathing = (1.0f - alpha) * ctrl->state.temporal_breathing + alpha * target_breathing;
    if (ctrl->state.temporal_breathing < 0.001f) ctrl->state.temporal_breathing = 1.0f;

    float breathing_factor = ctrl->state.temporal_breathing;
    if (ctrl->state.descent_phase) {
        breathing_factor = 0.35f;
    } else if (ctrl->state.landing_phase) {
        breathing_factor = 0.55f;
    }

    float dx_tgt = current_pos[0] - target_pos[0];
    float dy_tgt = current_pos[1] - target_pos[1];
    float att_norm = norm2f(dx_tgt, dy_tgt);
    float k_att = ctrl->params.attraction_gain / fmaxf(0.35f, breathing_factor);

    // Normalized attraction toward target XY. Z is handled explicitly in control_output.
    gradient[0] += k_att * (dx_tgt / att_norm) * 3.5f;
    gradient[1] += k_att * (dy_tgt / att_norm) * 3.5f;

    float speed_current = norm3f(ctrl->state.current_vel[0], ctrl->state.current_vel[1], ctrl->state.current_vel[2]);
    float dynamic_safety_buffer = ctrl->params.safety_radius * (1.0f + 0.20f * speed_current);
    float k_repu_base = ctrl->params.mode_v * breathing_factor;

    // Once landing, target commitment should dominate over obstacle points projected near the pad.
    float pad_ignore_radius = ctrl->state.descent_phase ? 2.0f : (ctrl->state.landing_phase ? 1.6f : 1.1f);

    for (int i = 0; i < num_obstacles; i++) {
        float ox = obstacles[i][0];
        float oy = obstacles[i][1];
        float oz = obstacles[i][2];
        float or = obstacles[i][3];

        if (oz >= 990.0f || (ox == 0.0f && oy == 0.0f && oz == 0.0f)) continue;

        float obs_to_tgt_xy = norm2f(ox - target_pos[0], oy - target_pos[1]);
        if (obs_to_tgt_xy < pad_ignore_radius) continue;

        float dx = current_pos[0] - ox;
        float dy = current_pos[1] - oy;
        float dz = current_pos[2] - oz;
        float dist = norm3f(dx, dy, dz);
        float effective_safety_radius = (dynamic_safety_buffer + or) * breathing_factor;

        if (dist < effective_safety_radius && dist > 0.01f) {
            float inv_r = 1.0f / effective_safety_radius;
            float inv_d = 1.0f / dist;
            float vertical_boost = (dz > -0.2f) ? 3.0f : 1.0f;
            float scalar = k_repu_base * (inv_r - inv_d) * (inv_d * inv_d * inv_d);
            if (scalar < -14000.0f) scalar = -14000.0f;

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
    float current_z = ctrl->state.current_pos[2];
    float tgt_z = ctrl->state.target_pos[2];
    float dx_to_pad = ctrl->state.target_pos[0] - ctrl->state.current_pos[0];
    float dy_to_pad = ctrl->state.target_pos[1] - ctrl->state.current_pos[1];
    float dist_xy_lock = norm2f(dx_to_pad, dy_to_pad);
    float z_dist_lock = current_z - tgt_z;

    float speed_current = norm3f(ctrl->state.current_vel[0], ctrl->state.current_vel[1], ctrl->state.current_vel[2]);
    float vxy_rel = norm2f(ctrl->state.current_vel[0] - ctrl->params.platform_vel_est[0],
                           ctrl->state.current_vel[1] - ctrl->params.platform_vel_est[1]);

    float desired_vx = -gradient[0];
    float desired_vy = -gradient[1];
    float desired_vz = -gradient[2];
    float max_total_speed = MAX_SPEED_PHYSICAL;

    bool in_approach = ctrl->state.landing_phase || ctrl->state.descent_phase;

    // ── Predictive escape outside final approach only ───────────────────────
    if (!in_approach) {
        float raw_threat = (ctrl->state.collision_risk_score * 0.50f) +
                           (ctrl->state.clutter_density * 0.30f) +
                           ((1.0f - ctrl->state.navigability_score) * 0.20f);
        ctrl->state.predictive_threat_score = 0.70f * ctrl->state.predictive_threat_score + 0.30f * raw_threat;
        if (ctrl->state.predictive_threat_score > 0.60f && speed_current < 0.7f) {
            ctrl->state.threat_accumulator += 1.0f;
        } else if (speed_current > 1.2f) {
            ctrl->state.threat_accumulator -= 2.0f;
        } else {
            ctrl->state.threat_accumulator -= 0.5f;
        }
        if (ctrl->state.threat_accumulator < 0.0f) ctrl->state.threat_accumulator = 0.0f;

        if (ctrl->state.threat_accumulator > 22.0f && !ctrl->state.predictive_escape_active
            && !ctrl->state.escape_mode_active) {
            ctrl->state.predictive_escape_active = true;
            ctrl->state.predictive_escape_ticks = 0;
            ctrl->state.predictive_escape_target_z = current_z + 1.0f + ctrl->state.clutter_density * 1.8f;
            ctrl->state.threat_accumulator = 0.0f;
        }
    }

    if (ctrl->state.predictive_escape_active) {
        ctrl->state.predictive_escape_ticks++;
        float climb_err = ctrl->state.predictive_escape_target_z - current_z;
        desired_vz = clampf_local(climb_err * 1.8f, 0.0f, 1.8f);
        if (climb_err < 0.15f || ctrl->state.predictive_escape_ticks > 220) {
            ctrl->state.predictive_escape_active = false;
            ctrl->state.predictive_escape_ticks = 0;
        }
    }

    // ── Reactive escape outside final approach only ─────────────────────────
    if (!in_approach && speed_current < 0.15f && dist_xy_lock > 0.20f) {
        ctrl->state.stuck_counter++;
    } else if (ctrl->state.stuck_counter > 0) {
        ctrl->state.stuck_counter--;
    }

    if (!in_approach && ctrl->state.stuck_counter > 75 && !ctrl->state.escape_mode_active
        && !ctrl->state.predictive_escape_active) {
        ctrl->state.escape_mode_active = true;
        float grad_norm = norm2f(gradient[0], gradient[1]);
        ctrl->state.escape_vector[0] = -gradient[1] / grad_norm;
        ctrl->state.escape_vector[1] =  gradient[0] / grad_norm;
        ctrl->state.escape_vector[2] = 1.4f;
    }

    if (ctrl->state.escape_mode_active) {
        if (speed_current > 1.0f || ctrl->state.stuck_counter > 150 || in_approach) {
            ctrl->state.escape_mode_active = false;
            ctrl->state.stuck_counter = 0;
        } else {
            desired_vx = ctrl->state.escape_vector[0] * 2.0f;
            desired_vy = ctrl->state.escape_vector[1] * 2.0f;
            desired_vz = ctrl->state.escape_vector[2];
        }
    }

    // ── Landing state machine ───────────────────────────────────────────────
    if (!ctrl->state.landing_phase) {
        if (dist_xy_lock < 3.0f && current_z > tgt_z + 0.20f) {
            ctrl->state.landing_phase = true;
            ctrl->state.predictive_escape_active = false;
            ctrl->state.escape_mode_active = false;
            ctrl->state.stuck_counter = 0;
        }
    } else if (!ctrl->state.descent_phase && dist_xy_lock > 5.5f) {
        ctrl->state.landing_phase = false;
    }

    if (ctrl->state.landing_phase && !ctrl->state.descent_phase) {
        bool xy_ok = dist_xy_lock < fmaxf(0.85f, ctrl->params.landing_threshold_xy * 1.65f);
        bool low_pass_ok = (ctrl->state.agl < 1.45f && dist_xy_lock < 1.35f);
        bool alt_ok = z_dist_lock > 0.18f;
        bool vxy_ok = vxy_rel < 1.65f;
        if ((xy_ok || low_pass_ok) && alt_ok && vxy_ok) {
            ctrl->state.descent_phase = true;
            ctrl->state.descent_vz = ctrl->state.current_vel[2];
        }
    }

    if (ctrl->state.descent_phase) {
        float h_rem = fmaxf(current_z - tgt_z, 0.0f);
        float descent_rate = clampf_local(ctrl->params.landing_descent_rate, 0.28f, 0.80f);
        float vz_target;
        if (current_z < tgt_z + 0.015f) {
            vz_target = 0.0f;
            ctrl->state.target_reached = (dist_xy_lock < 0.55f && fabsf(ctrl->state.current_vel[2]) < 0.45f);
        } else if (h_rem > 0.75f) {
            vz_target = -descent_rate;
        } else if (h_rem > 0.25f) {
            vz_target = -fmaxf(0.30f, descent_rate * 0.75f);
        } else {
            vz_target = -fmaxf(0.18f, descent_rate * 0.45f);
        }
        vz_target += ctrl->params.platform_vel_est[2];
        desired_vz = clampf_local((vz_target - ctrl->state.current_vel[2]) * 1.7f + vz_target, -1.05f, 0.55f);

        float d_to_pad = norm2f(dx_to_pad, dy_to_pad);
        float speed_xy = clampf_local(dist_xy_lock * 1.25f, 0.04f, 0.70f);
        if (dist_xy_lock < 0.16f) {
            desired_vx = ctrl->params.platform_vel_est[0];
            desired_vy = ctrl->params.platform_vel_est[1];
        } else {
            desired_vx = (dx_to_pad / d_to_pad) * speed_xy + ctrl->params.platform_vel_est[0];
            desired_vy = (dy_to_pad / d_to_pad) * speed_xy + ctrl->params.platform_vel_est[1];
        }
        max_total_speed = 1.05f;
    } else if (ctrl->state.landing_phase) {
        float d_to_pad = norm2f(dx_to_pad, dy_to_pad);
        float speed_xy = clampf_local(dist_xy_lock * 1.10f, 0.10f, 1.25f);
        desired_vx = (dx_to_pad / d_to_pad) * speed_xy + ctrl->params.platform_vel_est[0] * 0.95f;
        desired_vy = (dy_to_pad / d_to_pad) * speed_xy + ctrl->params.platform_vel_est[1] * 0.95f;

        float hover_offset = 0.35f + 0.55f * jnp_clip_placeholder(dist_xy_lock / 2.0f);
        float target_z_ref = tgt_z + hover_offset;
        desired_vz = clampf_local((target_z_ref - current_z) * 1.8f, -0.65f, 0.85f);
        max_total_speed = 1.55f;
    } else {
        float grad_norm = norm2f(desired_vx, desired_vy);
        float speed_xy = 2.25f;
        if (dist_xy_lock < 20.0f) {
            speed_xy = 1.9f + jnp_clip_placeholder((dist_xy_lock - 4.0f) / 16.0f) * 0.55f;
        }
        desired_vx = (desired_vx / grad_norm) * speed_xy + ctrl->params.platform_vel_est[0] * 0.85f;
        desired_vy = (desired_vy / grad_norm) * speed_xy + ctrl->params.platform_vel_est[1] * 0.85f;
        desired_vz += ctrl->params.attraction_gain * 1.8f * (ctrl->params.cruise_altitude - current_z);
        desired_vz = clampf_local(desired_vz, -1.5f, 1.7f);
        max_total_speed = MAX_SPEED_PHYSICAL;
    }

    if (ctrl->state.landing_phase) {
        float damping_factor = clampf_local(0.45f * (1.0f - dist_xy_lock / 2.5f), 0.0f, 0.45f);
        desired_vx -= damping_factor * (ctrl->state.current_vel[0] - ctrl->params.platform_vel_est[0]);
        desired_vy -= damping_factor * (ctrl->state.current_vel[1] - ctrl->params.platform_vel_est[1]);
    }

    float total_speed = norm3f(desired_vx, desired_vy, desired_vz);
    if (total_speed > max_total_speed) {
        desired_vx = (desired_vx / total_speed) * max_total_speed;
        desired_vy = (desired_vy / total_speed) * max_total_speed;
        desired_vz = (desired_vz / total_speed) * max_total_speed;
    }

    // Rate limiting. Descent/landing receive more vertical authority to avoid 60s timeouts.
    float max_dv = 2.4f * SIM_DT;
    float max_dv_z = max_dv;
    if (ctrl->state.descent_phase) {
        max_dv_z = max_dv * 8.0f;
    } else if (ctrl->state.landing_phase) {
        max_dv_z = max_dv * 4.0f;
    }

    if (!ctrl->state.descent_phase && current_z < 0.15f) {
        desired_vz = 1.8f;
        ctrl->state.last_vel_cmd[2] = 1.8f;
    }

    float dv_x = desired_vx - ctrl->state.last_vel_cmd[0];
    float dv_y = desired_vy - ctrl->state.last_vel_cmd[1];
    float dv_z = desired_vz - ctrl->state.last_vel_cmd[2];
    float dv_xy_norm = norm2f(dv_x, dv_y);
    if (dv_xy_norm > max_dv) {
        dv_x = (dv_x / dv_xy_norm) * max_dv;
        dv_y = (dv_y / dv_xy_norm) * max_dv;
    }
    if (fabsf(dv_z) > max_dv_z) {
        dv_z = (dv_z >= 0.0f ? 1.0f : -1.0f) * max_dv_z;
    }

    ctrl->state.control_output[0] = ctrl->state.last_vel_cmd[0] + dv_x;
    ctrl->state.control_output[1] = ctrl->state.last_vel_cmd[1] + dv_y;
    ctrl->state.control_output[2] = ctrl->state.last_vel_cmd[2] + dv_z;
    memcpy(ctrl->state.last_vel_cmd, ctrl->state.control_output, sizeof(float) * 3);
    ctrl->state.control_output[3] = norm3f(ctrl->state.control_output[0], ctrl->state.control_output[1], ctrl->state.control_output[2]);

    float desired_yaw = ctrl->state.current_rpy[2];
    if (dist_xy_lock > 0.15f) {
        desired_yaw = atan2f(ctrl->state.target_pos[1] - ctrl->state.current_pos[1],
                             ctrl->state.target_pos[0] - ctrl->state.current_pos[0]);
    }
    float yaw_error_wrapped = atan2f(sinf(desired_yaw - ctrl->state.current_rpy[2]),
                                     cosf(desired_yaw - ctrl->state.current_rpy[2]));
    ctrl->state.control_output[4] = clampf_local((yaw_error_wrapped - (0.15f * ctrl->state.yaw_rate)) / (float)M_PI, -1.0f, 1.0f);
}

adaptation_controller_t* adapt_init()
{
    adaptation_controller_t *controller = (adaptation_controller_t*)malloc(sizeof(adaptation_controller_t));
    if (!controller) return NULL;
    memset(controller, 0, sizeof(adaptation_controller_t));

    controller->params.cruise_altitude = 1.5f;
    controller->params.safety_radius = 1.10f;
    controller->params.mode_v = 160.0f;
    controller->params.attraction_gain = 11.0f;
    controller->params.max_speed = 3.0f;
    controller->params.max_yaw_rate = 1.5f;
    controller->params.landing_descent_rate = 0.45f;
    controller->params.landing_threshold_xy = 0.65f;
    controller->params.landing_threshold_z = 1.20f;
    memset(controller->params.platform_vel_est, 0, sizeof(controller->params.platform_vel_est));

    controller->state.mode = ADAPT_MODE_OFF;
    controller->state.speed = ADAPT_SPEED_MEDIUM;
    controller->state.temporal_breathing = 1.0f;
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
    controller->state.yaw_rate = 0.0f;
    controller->state.agl = 0.0f;
    controller->state.landing_phase = false;
    controller->state.descent_phase = false;
    controller->state.descent_vz = 0.0f;
    controller->state.collision_detected = false;
    controller->state.target_reached = false;
    controller->state.stuck_counter = 0;
    controller->state.escape_mode_active = false;
    memset(controller->state.escape_vector, 0, sizeof(float) * 3);
    controller->state.predictive_threat_score = 0.0f;
    controller->state.threat_accumulator = 0.0f;
    controller->state.predictive_escape_active = false;
    controller->state.predictive_escape_target_z = 0.0f;
    controller->state.predictive_escape_ticks = 0;
    controller->state.clutter_density = 0.0f;
    controller->state.navigability_score = 1.0f;
    controller->state.collision_risk_score = 0.0f;
    controller->state.temporal_breathing = 1.0f;
    controller->state.convergence = 0.0f;
    controller->state.converged = false;
    controller->state.iterations = 0;
    controller->num_internal_obstacles = 0;
}

void adapt_set_flight_params(adaptation_controller_t *controller, const adapt_flight_params_t *params)
{
    if (!controller || !params) return;
    memcpy(&controller->params, params, sizeof(adapt_flight_params_t));
    controller->params.max_speed = clampf_local(controller->params.max_speed, 0.1f, MAX_SPEED_PHYSICAL);
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

    float gradient[3];
    calculate_potential_gradient(controller,
                                 controller->state.current_pos,
                                 controller->state.target_pos,
                                 controller->internal_obstacles,
                                 controller->num_internal_obstacles,
                                 gradient);

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

    controller->num_internal_obstacles = 0;

    psmsl_depth_result_t depth_analysis_result;
    depth_analysis_result.clutter_density = 0.0f;
    depth_analysis_result.local_navigability = 1.0f;
    depth_analysis_result.collision_risk_score = 0.0f;

    if (depth_image && depth_width > 0 && depth_height > 0) {
        float depth_search_radius = max_range * 0.75f;
        psmsl_depth_analyze_image(depth_image, depth_width, depth_height,
                                  controller->state.current_pos, controller->state.current_rpy,
                                  max_range, fov_deg, depth_search_radius,
                                  controller->internal_obstacles, &controller->num_internal_obstacles,
                                  ADAPT_MAX_OBSTACLES, &depth_analysis_result);
    }

    controller->state.clutter_density = depth_analysis_result.clutter_density;
    controller->state.navigability_score = depth_analysis_result.local_navigability;
    controller->state.collision_risk_score = depth_analysis_result.collision_risk_score;

    float dx = controller->state.current_pos[0] - controller->state.target_pos[0];
    float dy = controller->state.current_pos[1] - controller->state.target_pos[1];
    float dz = controller->state.current_pos[2] - controller->state.target_pos[2];
    float pos_err = norm3f(dx, dy, dz);
    float vel_mag = norm3f(controller->state.current_vel[0], controller->state.current_vel[1], controller->state.current_vel[2]);
    controller->state.convergence = pos_err;
    controller->state.converged = (pos_err < CONVERGENCE_THRESHOLD_POS && vel_mag < CONVERGENCE_THRESHOLD_VEL);
}

const adapt_state_t* adapt_get_state(const adaptation_controller_t *controller) { return controller ? &controller->state : NULL; }

int adapt_export_state_json(const adaptation_controller_t *controller, char *buffer, size_t buffer_size)
{
    if (!controller || !buffer || buffer_size == 0) return 0;
    return snprintf(buffer, buffer_size,
                    "{\"mode\":%d,\"current_pos\":[%.2f,%.2f,%.2f],\"landing_phase\":%d,\"descent_phase\":%d,\"collision_risk\":%.2f}",
                    controller->state.mode,
                    controller->state.current_pos[0],
                    controller->state.current_pos[1],
                    controller->state.current_pos[2],
                    controller->state.landing_phase ? 1 : 0,
                    controller->state.descent_phase ? 1 : 0,
                    controller->state.collision_risk_score);
}
