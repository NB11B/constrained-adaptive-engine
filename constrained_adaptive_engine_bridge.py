import ctypes
import os
import numpy as np

# Define the path to the shared library
_library_path = os.path.join(os.path.dirname(__file__), 'libadaptive_controller.so')
_adaptive_engine = ctypes.CDLL(_library_path)

# =============================================================================
# C Data Structures (Mirroring C definitions)
# =============================================================================

# adapt_params_t
class AdaptParams(ctypes.Structure):
    _fields_ = [
        ('max_speed', ctypes.c_float),
        ('safety_radius', ctypes.c_float),
        ('mode_v', ctypes.c_float),
        ('platform_vel_est', ctypes.c_float * 3),
    ]

# adapt_state_t
class AdaptState(ctypes.Structure):
    _fields_ = [
        ('mode', ctypes.c_int),  # adapt_mode_t
        ('speed', ctypes.c_int), # adapt_speed_t
        ('current_pos', ctypes.c_float * 3),
        ('current_vel', ctypes.c_float * 3),
        ('current_rpy', ctypes.c_float * 3),
        ('target_pos', ctypes.c_float * 3),
        ('yaw_rate', ctypes.c_float),
        ('agl', ctypes.c_float),
        ('last_vel_cmd', ctypes.c_float * 3),
        ('control_output', ctypes.c_float * 5), # vx, vy, vz, total_speed, yaw_rate_cmd
        ('landing_phase', ctypes.c_bool),
        ('descent_phase', ctypes.c_bool),
        ('descent_vz', ctypes.c_float),
        ('collision_detected', ctypes.c_bool),
        ('target_reached', ctypes.c_bool),
        ('clutter_density', ctypes.c_float), # PSMSL result
        ('local_navigability', ctypes.c_float), # PSMSL result
        ('collision_risk_score', ctypes.c_float), # PSMSL result
        ('convergence', ctypes.c_float),
        ('converged', ctypes.c_bool),
        ('iterations', ctypes.c_uint32),
        ('timestamp_ms', ctypes.c_uint32),
    ]

# adaptation_controller_t (opaque pointer in Python)
class AdaptationController(ctypes.Structure):
    pass # Opaque structure

# =============================================================================
# C Function Prototypes
# =============================================================================

# adapt_init
_adaptive_engine.adapt_init.argtypes = []
_adaptive_engine.adapt_init.restype = ctypes.POINTER(AdaptationController)

# adapt_free
_adaptive_engine.adapt_free.argtypes = [ctypes.POINTER(AdaptationController)]
_adaptive_engine.adapt_free.restype = None

# adapt_set_flight_params
_adaptive_engine.adapt_set_flight_params.argtypes = [
    ctypes.POINTER(AdaptationController),
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.POINTER(ctypes.c_float * 3)
]
_adaptive_engine.adapt_set_flight_params.restype = None

# adapt_set_target_pos
_adaptive_engine.adapt_set_target_pos.argtypes = [
    ctypes.POINTER(AdaptationController),
    ctypes.POINTER(ctypes.c_float * 3)
]
_adaptive_engine.adapt_set_target_pos.restype = None

# adapt_process_sensor_data
_adaptive_engine.adapt_process_sensor_data.argtypes = [
    ctypes.POINTER(AdaptationController),
    ctypes.POINTER(ctypes.c_float * 3), # current_pos
    ctypes.POINTER(ctypes.c_float * 3), # current_vel
    ctypes.POINTER(ctypes.c_float * 3), # current_rpy
    ctypes.POINTER(ctypes.c_float * 3), # target_pos
    ctypes.c_float,                     # yaw_rate
    ctypes.c_float,                     # agl
    ctypes.POINTER(ctypes.c_float * 4), # obstacles (array of [x,y,z,r])
    ctypes.c_int                        # num_obstacles
]
_adaptive_engine.adapt_process_sensor_data.restype = None

# adapt_update
_adaptive_engine.adapt_update.argtypes = [ctypes.POINTER(AdaptationController)]
_adaptive_engine.adapt_update.restype = ctypes.c_bool

# adapt_get_state
_adaptive_engine.adapt_get_state.argtypes = [ctypes.POINTER(AdaptationController)]
_adaptive_engine.adapt_get_state.restype = ctypes.POINTER(AdaptState)

# adapt_export_state_json
_adaptive_engine.adapt_export_state_json.argtypes = [
    ctypes.POINTER(AdaptationController),
    ctypes.c_char_p,
    ctypes.c_size_t
]
_adaptive_engine.adapt_export_state_json.restype = ctypes.c_int

# =============================================================================
# Python Wrapper Class
# =============================================================================

class ConstrainedAdaptiveEngine:
    def __init__(self):
        self._controller = _adaptive_engine.adapt_init()
        if not self._controller:
            raise RuntimeError("Failed to initialize C adaptive engine.")

    def __del__(self):
        if self._controller:
            _adaptive_engine.adapt_free(self._controller)
            self._controller = None

    def set_flight_params(self, max_speed, safety_radius, mode_v, platform_vel_est):
        platform_vel_est_c = (ctypes.c_float * 3)(*platform_vel_est)
        _adaptive_engine.adapt_set_flight_params(
            self._controller, max_speed, safety_radius, mode_v, platform_vel_est_c
        )

    def set_target_pos(self, target_pos):
        target_pos_c = (ctypes.c_float * 3)(*target_pos)
        _adaptive_engine.adapt_set_target_pos(self._controller, target_pos_c)

    def process_sensor_data(self, current_pos, current_vel, current_rpy, target_pos, yaw_rate, agl, obstacles, num_obstacles):
        current_pos_c = (ctypes.c_float * 3)(*current_pos)
        current_vel_c = (ctypes.c_float * 3)(*current_vel)
        current_rpy_c = (ctypes.c_float * 3)(*current_rpy)
        target_pos_c = (ctypes.c_float * 3)(*target_pos)
        
        # Create a 2D array of obstacles for C: float obstacles[][4]
        ObstaclesArray = (ctypes.c_float * 4) * num_obstacles
        obstacles_c = ObstaclesArray()
        for i in range(num_obstacles):
            for j in range(4):
                obstacles_c[i][j] = float(obstacles[i][j])

        _adaptive_engine.adapt_process_sensor_data(
            self._controller, current_pos_c, current_vel_c, current_rpy_c,
            target_pos_c, yaw_rate, agl, obstacles_c, num_obstacles
        )

    def start_adaptive(self):
        return _adaptive_engine.adapt_start_adaptive(self._controller)

    def update(self):
        return _adaptive_engine.adapt_update(self._controller)

    def get_state(self):
        state_ptr = _adaptive_engine.adapt_get_state(self._controller)
        if state_ptr:
            return state_ptr.contents
        return None

    def export_state_json(self, buffer_size=1024):
        buffer = ctypes.create_string_buffer(buffer_size)
        length = _adaptive_engine.adapt_export_state_json(self._controller, buffer, buffer_size)
        if length > 0:
            return buffer.value.decode('utf-8')
        return "{}"

# Example Usage (for testing)
if __name__ == '__main__':
    engine = ConstrainedAdaptiveEngine()
    print("Engine initialized.")

    # Set initial parameters
    engine.set_flight_params(max_speed=3.0, safety_radius=1.5, mode_v=1.0, platform_vel_est=[0.0, 0.0, 0.0])
    engine.set_target_pos(target_pos=[0.0, 0.0, 1.0])

    # Simulate some sensor data
    current_pos = [0.0, 0.0, 2.0]
    current_vel = [0.0, 0.0, 0.0]
    current_rpy = [0.0, 0.0, 0.0]
    target_pos = [0.0, 0.0, 1.0]
    yaw_rate = 0.0
    agl = 1.0
    obstacles = [[1.0, 1.0, 1.5, 0.2], [-1.0, -1.0, 1.8, 0.3]] # x,y,z,r
    num_obstacles = len(obstacles)

    engine.process_sensor_data(current_pos, current_vel, current_rpy, target_pos, yaw_rate, agl, obstacles, num_obstacles)
    engine.update()

    state = engine.get_state()
    if state:
        print(f"Current Pos: {list(state.current_pos)}")
        print(f"Target Pos: {list(state.target_pos)}")
        print(f"Control Output (vx,vy,vz,total_speed,yaw_cmd): {list(state.control_output)}")
        print(f"Clutter Density: {state.clutter_density}")
        print(f"Local Navigability: {state.local_navigability}")
        print(f"Collision Risk Score: {state.collision_risk_score}")
        print(f"Convergence: {state.convergence}")
        print(f"Converged: {state.converged}")
        print(f"JSON State: {engine.export_state_json()}")

    del engine
    print("Engine freed.")
