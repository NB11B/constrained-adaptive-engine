/**
 * @file adaptation_controller.h
 * @brief High-Performance MCU-Compliant Analytical Autopilot for Swarm Subnet 124
 */

#ifndef ADAPTATION_CONTROLLER_H
#define ADAPTATION_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Configuration Constants
// =============================================================================

#define ADAPT_UPDATE_RATE_HZ        50      // Standard PyBullet loop frequency
#define ADAPT_HISTORY_SIZE          64      // Smoothing buffer length
#define ADAPT_MAX_OBSTACLES         256     // Subnet 124 observation limit

// =============================================================================
// Datatypes & Enumerations
// =============================================================================

typedef enum {
    ADAPT_MODE_OFF = 0,
    ADAPT_MODE_CALIBRATE = 1,
    ADAPT_MODE_ADAPTIVE = 2,
    ADAPT_MODE_HOLD = 3
} adapt_mode_t;

typedef enum {
    ADAPT_SPEED_SLOW = 0,
    ADAPT_SPEED_MEDIUM = 1,
    ADAPT_SPEED_FAST = 2
} adapt_speed_t;

typedef struct {
    float cruise_altitude;
    float safety_radius;
    float mode_v;
    float attraction_gain;
    float max_speed;
    float max_yaw_rate;
    float landing_descent_rate;
    float landing_threshold_xy;
    float landing_threshold_z;
    float platform_vel_est[3];
} adapt_flight_params_t;

typedef struct {
    adapt_mode_t mode;
    adapt_speed_t speed;
    
    float current_pos[3];
    float current_vel[3];
    float current_rpy[3];
    float yaw_rate;
    float agl;
    float target_pos[3];
    float control_output[5]; // vx, vy, vz, total_speed, yaw_rate_cmd
    float last_vel_cmd[3];
    
    bool landing_phase;
    bool descent_phase;
    float descent_vz;
    
    bool collision_detected;
    bool target_reached;
    
    // Reactive local minimum escape state
    uint32_t stuck_counter;
    bool escape_mode_active;
    float escape_vector[3];

    // Predictive local minimum detector state
    float predictive_threat_score;    // Blended PSMSL threat scalar [0,1]
    float threat_accumulator;         // Leaky integrator over sustained threat
    bool predictive_escape_active;    // Predictive escape in progress
    float predictive_escape_target_z; // Target altitude for climb escape
    uint32_t predictive_escape_ticks; // Ticks since predictive escape started
    
    float clutter_density;      // PSMSL result
    float navigability_score;   // PSMSL result
    float collision_risk_score; // PSMSL result
    float temporal_breathing;   // Smoothed breathing factor
    
    float convergence;
    bool converged;
    uint32_t iterations;
    uint32_t timestamp_ms;
} adapt_state_t;

typedef struct {
    adapt_flight_params_t params;
    adapt_state_t state;
    float pos_history[ADAPT_HISTORY_SIZE][3];
    uint8_t history_index;
    float internal_obstacles[ADAPT_MAX_OBSTACLES][4];
    int num_internal_obstacles;
    uint32_t last_update_ms;
    uint32_t update_interval_ms;
    bool initialized;
    bool running;
} adaptation_controller_t;

#ifdef _WIN32
#define CAE_API __declspec(dllexport)
#else
#define CAE_API
#endif

// =============================================================================
// Public Functional Synapse Protocols
// =============================================================================

CAE_API adaptation_controller_t* adapt_init();
CAE_API void adapt_free(adaptation_controller_t *controller);
CAE_API void adapt_reset(adaptation_controller_t *controller);
CAE_API void adapt_set_flight_params(adaptation_controller_t *controller, const adapt_flight_params_t *params);
CAE_API void adapt_set_mode(adaptation_controller_t *controller, adapt_mode_t mode);
CAE_API void adapt_set_speed(adaptation_controller_t *controller, adapt_speed_t speed);
CAE_API void adapt_set_target_pos(adaptation_controller_t *controller, const float target_pos[3]);
CAE_API bool adapt_start_adaptive(adaptation_controller_t *controller);
CAE_API void adapt_stop(adaptation_controller_t *controller);
CAE_API bool adapt_update(adaptation_controller_t *controller);
CAE_API void adapt_process_sensor_data(adaptation_controller_t *controller,
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
                                   float fov_deg);
CAE_API const adapt_state_t* adapt_get_state(const adaptation_controller_t *controller);
CAE_API int adapt_export_state_json(const adaptation_controller_t *controller, char *buffer, size_t buffer_size);
float jnp_clip_placeholder(float val);

#ifdef __cplusplus
}
#endif

#endif // ADAPTATION_CONTROLLER_H
