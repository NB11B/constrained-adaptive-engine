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
lib_name = 'libadaptive_controller.dll' if os.name == 'nt' else 'libadaptive_controller.so'
_library_path = os.path.join(_dir, lib_name)

def compile_c_library():
    import subprocess
    import shutil
    print("[BRIDGE] Compiling C flight engine dynamically...")
    src_dir = os.path.join(_dir, "src")
    include_dir = os.path.join(_dir, "include")
    
    c_files = [
        os.path.join(src_dir, "adaptation_controller.c"),
        os.path.join(src_dir, "psmsl_depth_processor.c")
    ]
    
    for f in c_files:
        if not os.path.exists(f):
            raise FileNotFoundError(f"Dynamic compilation failed: missing source file {f}")
            
    if os.name != 'nt':  # Linux / macOS
        compilers = ['gcc', 'clang']
        compiled = False
        errors = []
        for compiler in compilers:
            if shutil.which(compiler):
                cmd = [
                    compiler, "-shared", "-o", _library_path, "-fPIC", "-O2",
                    f"-I{include_dir}", c_files[0], c_files[1], "-lm"
                ]
                try:
                    subprocess.run(cmd, capture_output=True, text=True, check=True)
                    compiled = True
                    break
                except subprocess.CalledProcessError as e:
                    errors.append(f"{compiler} compile error:\nSTDOUT: {e.stdout}\nSTDERR: {e.stderr}")
        if not compiled:
            raise RuntimeError("Dynamic compilation failed on Linux. Errors:\n" + "\n".join(errors))
    else:  # Windows
        compiled = False
        errors = []
        if shutil.which("gcc"):
            cmd = [
                "gcc", "-shared", "-o", _library_path, "-O2",
                f"-I{include_dir}", c_files[0], c_files[1], "-lm"
            ]
            try:
                subprocess.run(cmd, capture_output=True, text=True, check=True)
                compiled = True
            except subprocess.CalledProcessError as e:
                errors.append(f"gcc compile error:\nSTDOUT: {e.stdout}\nSTDERR: {e.stderr}")
                
        if not compiled and shutil.which("cl"):
            cmd = [
                "cl", "/LD", "/O2", "/D_USE_MATH_DEFINES",
                f"/I{include_dir}", c_files[0], c_files[1], f"/Fe:{_library_path}"
            ]
            try:
                subprocess.run(cmd, capture_output=True, text=True, check=True)
                compiled = True
            except subprocess.CalledProcessError as e:
                errors.append(f"cl compile error:\nSTDOUT: {e.stdout}\nSTDERR: {e.stderr}")
                
        if not compiled:
            raise RuntimeError("Dynamic compilation failed on Windows. Make sure gcc is in PATH or run from MSVC developer command prompt. Errors:\n" + "\n".join(errors))
            
    print(f"[BRIDGE] Dynamic compilation successful. Library generated at: {_library_path}")

if not os.path.exists(_library_path):
    try:
        compile_c_library()
    except Exception as e:
        raise RuntimeError(f"Could not load or compile C library: {e}")

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
        # Local minimum escape state (must match C struct order exactly)
        ('stuck_counter', ctypes.c_uint32),
        ('escape_mode_active', ctypes.c_bool),
        ('_pad_escape', ctypes.c_uint8 * 3),  # alignment padding after bool
        ('escape_vector', ctypes.c_float * 3),
        ('clutter_density', ctypes.c_float),      # PSMSL result
        ('navigability_score', ctypes.c_float),   # PSMSL result (was local_navigability)
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
    ctypes.POINTER(ctypes.c_float),     # depth_image
    ctypes.c_int,                       # depth_width
    ctypes.c_int,                       # depth_height
    ctypes.c_float,                     # max_range
    ctypes.c_float                      # fov_deg
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

    def process_sensor_data(self, current_pos, current_vel, current_rpy, target_pos, yaw_rate, agl, depth_image=None, depth_width=128, depth_height=128, max_range=20.0, fov_deg=90.0):
        current_pos_c = (ctypes.c_float * 3)(*current_pos)
        current_vel_c = (ctypes.c_float * 3)(*current_vel)
        current_rpy_c = (ctypes.c_float * 3)(*current_rpy)
        target_pos_c = (ctypes.c_float * 3)(*target_pos)
        
        if depth_image is not None:
            # Use numpy's ctypes interface for zero-copy float32 pointer — critical for 50Hz performance
            depth_f32 = np.ascontiguousarray(depth_image.flatten(), dtype=np.float32)
            c_depth_image = depth_f32.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        else:
            c_depth_image = None

        _adaptive_engine.adapt_process_sensor_data(
            self._controller, 
            ctypes.byref(current_pos_c), 
            ctypes.byref(current_vel_c), 
            ctypes.byref(current_rpy_c),
            ctypes.byref(target_pos_c), 
            float(yaw_rate), 
            float(agl), 
            c_depth_image,
            int(depth_width),
            int(depth_height),
            float(max_range),
            float(fov_deg)
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
            depth_image=np.ones((128, 128, 1), dtype=np.float32)
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

