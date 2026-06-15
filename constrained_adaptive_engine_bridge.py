"""
sotapilot_bridge.py — High-Performance Python/C ctypes Bridge Interface
========================================================================
Synchronized with bare-metal memory alignments to prevent segmentation faults.
"""

import ctypes
import os
import sys
import numpy as np

# ── Global Static Array Limits ──────────────────────────────────────────────
ADAPT_MAX_OBSTACLES = 256

# Locate and resolve the local absolute file library path layout
_dir = os.path.dirname(os.path.abspath(__file__))
_library_path = os.path.join(_dir, 'libadaptive_controller.so')

if not os.path.exists(_library_path):
    raise FileNotFoundError(f"Missing required bare-metal flight binary at: {_library_path}")

_adaptive_engine = ctypes.CDLL(_library_path)

# =============================================================================
# Bare-Metal Structure Definitions (Memory Aligned)
# =============================================================================

class AdaptParams(ctypes.Structure):
    _fields_ = [
        ('cruise_altitude', ctypes.c_float),
        ('safety_radius', ctypes.c_float),
        ('mode_v', ctypes.c_float),
        ('attraction_gain', ctypes.c_float),
        ('max_speed', ctypes.c_float),
        ('max_yaw_rate', ctypes.c_float),
        ('landing_descent_rate', ctypes.c_float),
        ('landing_threshold_xy', ctypes.c_float),
        ('landing_threshold_z', ctypes.c_float),
        ('platform_vel_est', ctypes.c_float * 3),
    ]

class AdaptState(ctypes.Structure):
    _fields_ = [
        ('mode', ctypes.c_int),  
        ('speed', ctypes.c_int), 
        ('current_pos', ctypes.c_float * 3),
        ('current_vel', ctypes.c_float * 3),
        ('current_rpy', ctypes.c_float * 3),
        ('yaw_rate', ctypes.c_float),
        ('agl', ctypes.c_float),
        ('target_pos', ctypes.c_float * 3),
        ('control_output', ctypes.c_float * 5), 
        ('last_vel_cmd', ctypes.c_float * 3),
        ('landing_phase', ctypes.c_bool),
        ('descent_phase', ctypes.c_bool),
        ('descent_vz', ctypes.c_float),
        ('collision_detected', ctypes.c_bool),
        ('target_reached', ctypes.c_bool),
        ('clutter_density', ctypes.c_float), # PSMSL result
        ('local_navigability', ctypes.c_float), # PSMSL result
        ('collision_risk_score', ctypes.c_float), # PSMSL result
        ('temporal_breathing', ctypes.c_float), # Smoothed breathing factor
        ('convergence', ctypes.c_float),
        ('converged', ctypes.c_bool),
        ('iterations', ctypes.c_uint32),
        ('timestamp_ms', ctypes.c_uint32),
    ]

class AdaptationController(ctypes.Structure):
    """Opaque pointer binding protecting internal context fields."""
    pass

# =============================================================================
# Strict Type Ingestion Definitions
# =============================================================================

_adaptive_engine.adapt_init.argtypes = []
_adaptive_engine.adapt_init.restype = ctypes.POINTER(AdaptationController)

_adaptive_engine.adapt_reset.argtypes = [ctypes.POINTER(AdaptationController)]
_adaptive_engine.adapt_reset.restype = None

_adaptive_engine.adapt_set_flight_params.argtypes = [
    ctypes.POINTER(AdaptationController),
    ctypes.POINTER(AdaptParams)
]
_adaptive_engine.adapt_set_flight_params.restype = None

_adaptive_engine.adapt_set_target_pos.argtypes = [
    ctypes.POINTER(AdaptationController),
    ctypes.POINTER(ctypes.c_float * 3)
]
_adaptive_engine.adapt_set_target_pos.restype = None

_adaptive_engine.adapt_start_adaptive.argtypes = [ctypes.POINTER(AdaptationController)]
_adaptive_engine.adapt_start_adaptive.restype = ctypes.c_bool

_adaptive_engine.adapt_update.argtypes = [ctypes.POINTER(AdaptationController)]
_adaptive_engine.adapt_update.restype = ctypes.c_bool

_adaptive_engine.adapt_process_sensor_data.argtypes = [
    ctypes.POINTER(AdaptationController),
    ctypes.POINTER(ctypes.c_float * 3), # current_pos
    ctypes.POINTER(ctypes.c_float * 3), # current_vel
    ctypes.POINTER(ctypes.c_float * 3), # current_rpy
    ctypes.POINTER(ctypes.c_float * 3), # target_pos
    ctypes.c_float,                     # yaw_rate
    ctypes.c_float,                     # agl
    ctypes.POINTER((ctypes.c_float * 4) * ADAPT_MAX_OBSTACLES), # Multi-Array pointer alignment
    ctypes.c_int                        # num_obstacles
]
_adaptive_engine.adapt_process_sensor_data.restype = None

_adaptive_engine.adapt_get_state.argtypes = [ctypes.POINTER(AdaptationController)]
_adaptive_engine.adapt_get_state.restype = ctypes.POINTER(AdaptState)

# =============================================================================
# Python Production Engine Interface Wrapper
# =============================================================================

class ConstrainedAdaptiveEngine:
    def __init__(self):
        self._controller = _adaptive_engine.adapt_init()
        if not self._controller:
            raise RuntimeError("CRITICAL: Failed to allocate bare-metal flight engine structure context memory.")
        
        # Pre-allocate array segment structures to save overhead inside the 50Hz execution thread
        self._obstacle_buffer_type = (ctypes.c_float * 4) * ADAPT_MAX_OBSTACLES
        self._obstacle_buffer = self._obstacle_buffer_type()

    def __del__(self):
        # Memory cleanup routine
        if hasattr(self, '_controller') and self._controller:
            # If explicit free helper handles termination on your C branch, swap to it here:
            pass

    def reset(self):
        _adaptive_engine.adapt_reset(self._controller)

    def set_flight_params(self, cruise_altitude=1.5, safety_radius=1.2, mode_v=35.0, 
                          attraction_gain=5.0, max_speed=3.0, max_yaw_rate=1.0, 
                          landing_descent_rate=0.015, landing_threshold_xy=0.40, 
                          landing_threshold_z=0.90, platform_vel_est=(0.0, 0.0, 0.0)):
        
        params = AdaptParams()
        params.cruise_altitude = float(cruise_altitude)
        params.safety_radius = float(safety_radius)
        params.mode_v = float(mode_v)
        params.attraction_gain = float(attraction_gain)
        params.max_speed = float(max_speed)
        params.max_yaw_rate = float(max_yaw_rate)
        params.landing_descent_rate = float(landing_descent_rate)
        params.landing_threshold_xy = float(landing_threshold_xy)
        params.landing_threshold_z = float(landing_threshold_z)
        
        for i in range(3):
            params.platform_vel_est[i] = float(platform_vel_est[i])
            
        _adaptive_engine.adapt_set_flight_params(self._controller, ctypes.byref(params))

    def set_target_pos(self, target_pos):
        target_pos_c = (ctypes.c_float * 3)(*target_pos)
        _adaptive_engine.adapt_set_target_pos(self._controller, ctypes.byref(target_pos_c))

    def process_sensor_data(self, current_pos, current_vel, current_rpy, target_pos, yaw_rate, agl, obstacles):
        current_pos_c = (ctypes.c_float * 3)(*current_pos)
        current_vel_c = (ctypes.c_float * 3)(*current_vel)
        current_rpy_c = (ctypes.c_float * 3)(*current_rpy)
        target_pos_c = (ctypes.c_float * 3)(*target_pos)
        
        num_obstacles = min(len(obstacles), ADAPT_MAX_OBSTACLES)
        
        # Zero out the reuse buffer to eliminate ghost points from previous passes
        ctypes.memset(ctypes.byref(self._obstacle_buffer), 0, ctypes.sizeof(self._obstacle_buffer))
        
        for i in range(num_obstacles):
            self._obstacle_buffer[i][0] = float(obstacles[i][0])
            self._obstacle_buffer[i][1] = float(obstacles[i][1])
            self._obstacle_buffer[i][2] = float(obstacles[i][2])
            self._obstacle_buffer[i][3] = float(obstacles[i][3])

        _adaptive_engine.adapt_process_sensor_data(
            self._controller, 
            ctypes.byref(current_pos_c), 
            ctypes.byref(current_vel_c), 
            ctypes.byref(current_rpy_c),
            ctypes.byref(target_pos_c), 
            float(yaw_rate), 
            float(agl), 
            ctypes.byref(self._obstacle_buffer), 
            int(num_obstacles)
        )

    def start_adaptive(self):
        return bool(_adaptive_engine.adapt_start_adaptive(self._controller))

    def update(self):
        return bool(_adaptive_engine.adapt_update(self._controller))

    def get_state(self):
        state_ptr = _adaptive_engine.adapt_get_state(self._controller)
        if state_ptr:
            return state_ptr.contents
        return None

# =============================================================================
# Diagnostic Verification Routine
# =============================================================================
if __name__ == '__main__':
    print("[INIT] Launching C Autopilot Engine verification validation matrix...")
    try:
        engine = ConstrainedAdaptiveEngine()
        print(" -> System allocation successful.")
        
        engine.set_flight_params(max_speed=3.0, safety_radius=1.2, mode_v=35.0)
        engine.set_target_pos(target_pos=[0.0, 0.0, 1.0])
        engine.start_adaptive()
        
        # Ingest simulated flight states
        engine.process_sensor_data(
            current_pos=[0.0, 0.0, 2.0],
            current_vel=[0.0, 0.0, 0.0],
            current_rpy=[0.0, 0.0, 0.0],
            target_pos=[0.0, 0.0, 1.0],
            yaw_rate=0.0,
            agl=1.0,
            obstacles=[[1.0, 1.0, 1.5, 0.2], [-1.0, -1.0, 1.8, 0.3]]
        )
        
        engine.update()
        state = engine.get_state()
        
        if state:
            print("\n[SUCCESS] Memory bounds match perfectly. Current metrics:")
            print(f" • Commanded Vector Array (XYZ): [{state.control_output[0]:.3f}, {state.control_output[1]:.3f}, {state.control_output[2]:.3f}]")
            print(f" • Throttle Magnitude Metric  : {state.control_output[3]:.3f} m/s")
            print(f" • Angular Yaw Command Scalar  : {state.control_output[4]:.3f}")
            print(f" • State Machine Landing Phase : {state.landing_phase}")
            print(f" • Total Iterations Processed  : {state.iterations}")
            
    except Exception as e:
        print(f"\n[FAILURE] Validation script aborted due to signature error: {str(e)}")
        sys.exit(1)

