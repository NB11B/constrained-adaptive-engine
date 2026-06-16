import numpy as np
from constrained_adaptive_engine_bridge import ConstrainedAdaptiveEngine

# ---------------------------------------------------------------------------
# Constants matching Swarm Subnet 4.0.2.9.5
# ---------------------------------------------------------------------------
SPEED_LIMIT      = 3.0   # m/s
MAX_RAY_DISTANCE = 30.0  # m
DEPTH_WIDTH      = 128
DEPTH_HEIGHT     = 128
CAMERA_FOV_DEG   = 90.0

# ---------------------------------------------------------------------------
# DroneFlightController (LITE VERSION)
# ---------------------------------------------------------------------------
class DroneFlightController:
    """
    Swarm Subnet 142 — Autonomous Drone Flight Controller (LITE)
    
    A streamlined, MCU-compliant controller using the AGL-based approach:
      1. Navigate to the noisy goal XY from the state vector.
      2. Use AGL rangefinder to detect when directly above the pad.
      3. Trigger the C-based CAE landing phase for precision touchdown.
    """

    def __init__(self):
        self._engine = ConstrainedAdaptiveEngine()
        self._initialized = False
        self._spawn_z = 1.8
        self._step = 0
        self._goal_xy = None
        self._goal_z = None

    def act(self, observation: dict) -> np.ndarray:
        """
        Compute flight action for the current observation.
        """
        state = observation["state"]
        depth = observation["depth"]

        # Unpack state vector
        pos      = state[0:3].astype(np.float64)
        rpy      = state[3:6].astype(np.float64)
        vel      = state[6:9].astype(np.float64)
        ang_vel  = state[9:12].astype(np.float64)
        yaw_rate = float(ang_vel[2])
        
        # AGL and search_vec from swarm-subnet 4.0.2.9.5 layout
        agl_norm = float(state[137])
        agl_m    = agl_norm * MAX_RAY_DISTANCE
        search_vec = state[138:141].astype(np.float64)
        
        # The search_vec points to the search_area_center (noisy estimate)
        noisy_goal = pos + search_vec
        self._goal_xy = noisy_goal[:2]
        self._goal_z = noisy_goal[2]

        # Initialize engine on first call
        if not self._initialized:
            self._spawn_z = float(pos[2])
            self._setup_engine(noisy_goal)
            self._initialized = True

        # Cruise altitude: at least 3m above spawn, and 2m above target
        cruise_alt = max(self._spawn_z + 3.0, float(self._goal_z) + 2.0)
        
        # Update C engine params
        # We use a tighter landing_threshold_xy (0.4m) for the AGL-based approach
        self._engine.set_flight_params(
            cruise_altitude      = cruise_alt,
            max_speed            = SPEED_LIMIT,
            safety_radius        = 1.15,
            mode_v               = 200.0,
            attraction_gain      = 8.0,      # Slightly higher gain for direct approach
            landing_threshold_xy = 0.40,     # Tight gate for AGL-triggered descent
            landing_threshold_z  = 1.50,
            landing_descent_rate = 0.025,
            platform_vel_est     = [0.0, 0.0, 0.0], # Static for now
        )

        # Feed sensor data to C engine
        self._engine.process_sensor_data(
            current_pos  = pos,
            current_vel  = vel,
            current_rpy  = rpy,
            target_pos   = noisy_goal,
            yaw_rate     = yaw_rate,
            agl          = agl_m,
            depth_image  = depth,
            depth_width  = DEPTH_WIDTH,
            depth_height = DEPTH_HEIGHT,
            max_range    = MAX_RAY_DISTANCE,
            fov_deg      = CAMERA_FOV_DEG,
        )

        # Run one control cycle
        self._engine.update()

        # Retrieve control output
        cs = self._engine.get_state()
        vx, vy, vz, total_speed, yaw_cmd = cs.control_output

        # Build action: unit direction + speed_norm
        vel_mag    = float(np.sqrt(vx**2 + vy**2 + vz**2)) + 1e-8
        dir_xyz    = np.array([vx, vy, vz], dtype=np.float32) / vel_mag
        speed_norm = float(np.clip(total_speed / SPEED_LIMIT, 0.0, 1.0))
        yaw_norm   = float(np.clip(yaw_cmd / np.pi, -1.0, 1.0))

        action = np.array(
            [dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_norm],
            dtype=np.float32,
        )

        self._step += 1
        return action

    def reset(self):
        """Reset controller state at mission start."""
        self._engine.reset()
        self._initialized = False
        self._spawn_z = 1.8
        self._step = 0
        self._goal_xy = None
        self._goal_z = None

    def _setup_engine(self, goal: np.ndarray):
        """Configure and start the C engine."""
        cruise_alt = max(self._spawn_z + 3.0, float(goal[2]) + 2.0)
        self._engine.set_flight_params(
            cruise_altitude      = cruise_alt,
            max_speed            = SPEED_LIMIT,
            safety_radius        = 1.15,
            mode_v               = 200.0,
            attraction_gain      = 8.0,
            landing_threshold_xy = 0.40,
            landing_threshold_z  = 1.50,
            landing_descent_rate = 0.025,
            platform_vel_est     = [0.0, 0.0, 0.0],
        )
        self._engine.set_target_pos(goal)
        self._engine.start_adaptive()

# Copyright Nathanael J. Bocker, 2026 all rights reserved
