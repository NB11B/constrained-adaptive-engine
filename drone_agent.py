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
        
        # AGL Calibration State
        self._calibrated_offset = np.zeros(3)
        self._is_calibrated = False
        self._search_start_step = 0
        self._search_phase = 0 # 0: approach, 1: spiral search, 2: locked
        self._agl_history = []
        self._takeoff_complete = False

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
        
        # Sanitize AGL (ignore impossible jumps)
        if hasattr(self, '_last_agl') and abs(agl_m - self._last_agl) > 5.0:
            agl_m = self._last_agl
        self._last_agl = agl_m
        
        search_vec = state[138:141].astype(np.float64)
        
        # 0. TAKEOFF PHASE
        if not self._takeoff_complete:
            if pos[2] > (self._spawn_z + 2.5):
                self._takeoff_complete = True
                print(f"[TAKEOFF] Altitude reached. Starting mission.")
        
        # 1. NOISE CANCELLATION VIA AGL CALIBRATION
        noisy_center = pos + search_vec
        dist_to_noisy = np.linalg.norm(pos[:2] - noisy_center[:2])
        
        # Detect AGL dip (pad is slightly raised, AGL will decrease relative to terrain)
        if not self._is_calibrated and dist_to_noisy < 5.0 and self._takeoff_complete:
            # More sensitive detection: look for a sudden dip of > 0.1m relative to recent average
            self._agl_history.append(agl_m)
            if len(self._agl_history) > 10: self._agl_history.pop(0)
            avg_agl = sum(self._agl_history) / len(self._agl_history)
            
            if agl_m < (avg_agl - 0.08) or agl_m < (pos[2] - 0.12):
                # Calibrate the XY offset, but keep Z at 0 as the pad is at a known height
                # When AGL dip is detected, the drone is currently over the true pad.
                # The calibrated offset is the difference between the noisy_center and the actual pad location (drone's current XY).
                # So, true_pad_location = noisy_center + self._calibrated_offset.
                # Therefore, self._calibrated_offset = pos - noisy_center.
                self._calibrated_offset = np.array([pos[0] - noisy_center[0], pos[1] - noisy_center[1], 0.0])
                self._is_calibrated = True
                print(f"[CALIBRATION] Pad detected via AGL dip! Offset locked at step {self._step}")

        # Apply calibration if found, otherwise use noisy center
        # true_target should be the actual goal position, corrected by the calibrated_offset.
        # Since calibrated_offset = pos - noisy_center, and we want to target the true pad,
        # true_target = noisy_center + calibrated_offset
        true_target = noisy_center + self._calibrated_offset

        # If not calibrated, perform a search pattern around the noisy target
        if not self._is_calibrated and dist_to_noisy < 5.0:
            # If we reached the noisy center but haven't seen the AGL dip, 
            # do a small search pattern at low altitude.
            if self._search_phase == 0:
                self._search_phase = 1
                self._search_start_step = self._step
            
            # Faster, tighter scan to cover the 3m uncertainty
            t = (self._step - self._search_start_step) * 0.15 # 3x faster
            # Cycle through 0.8m, 1.6m, 2.4m, 3.2m radii
            r = 0.8 * (1 + (np.floor(t / 5.0) % 4))
            search_offset = np.array([r * np.cos(t), r * np.sin(t), 0.0])
            true_target += search_offset
        elif self._is_calibrated and self._search_phase != 2:
            # Once calibrated, lock the search phase to prevent further searching
            self._search_phase = 2 # Locked
            print(f"[CALIBRATION] Search phase locked at step {self._step}")
        
        # After calibration, ensure true_target is solely based on the calibrated offset
        if self._is_calibrated:
            true_target = noisy_center - self._calibrated_offset

        # Initialize engine on first call
        if not self._initialized:
            self._spawn_z = float(pos[2])
            self._setup_engine()
            self._initialized = True
        
        # Always update C engine target position
        # Ensure the Z coordinate of the target is the pad height (0.2m)
        # so the C engine's landing logic triggers correctly.
        target_3d = np.array([true_target[0], true_target[1], 0.2], dtype=np.float64)
        self._engine.set_target_pos(target_3d)

        # Cruise altitude: lower for search, higher for cruise
        # We need to be low (1.2m above pad) to reliably detect the AGL dip
        search_alt = float(noisy_center[2]) + 1.2
        cruise_alt = max(self._spawn_z + 3.0, float(noisy_center[2]) + 2.5)
        
        # Trigger search altitude as we approach the noisy estimate
        in_search_zone = dist_to_noisy < 5.0
        
        if not self._is_calibrated:
            target_alt = search_alt if in_search_zone else cruise_alt
        else:
            # Once calibrated, descend to approach altitude to trigger C engine landing phase
            # Once calibrated, descend to approach altitude to trigger C engine landing phase
            # Target 1.0m above the noisy center (which is usually ground level), or 0.8m above pad.
            target_alt = 1.0
        
        # Force takeoff altitude
        if not self._takeoff_complete:
            target_alt = self._spawn_z + 3.0
        
        # Terrain-Following: Never go below 1.0m AGL during search
        if not self._is_calibrated and in_search_zone and self._takeoff_complete:
            target_alt = max(target_alt, pos[2] - agl_m + 1.0)
        
        # Update C engine params
        # We use a tighter landing_threshold_xy (0.4m) for the AGL-based approach
        self._engine.set_flight_params(
            cruise_altitude      = target_alt,
            max_speed            = SPEED_LIMIT,
            safety_radius        = 1.15,
            mode_v               = 200.0,
            attraction_gain      = 12.0,     # Increased gain for tighter centering
            landing_threshold_xy = 0.20,     # Very tight gate for AGL-triggered descent
            landing_threshold_z  = 1.50,
            landing_descent_rate = 0.025,
            platform_vel_est     = [0.0, 0.0, 0.0], # Static for now
        )

        # Feed sensor data to C engine
        # Clamp AGL to prevent death dive on sensor loss (30.0 = MAX_RANGE)
        safe_agl = min(agl_m, pos[2] + 0.5) if agl_m < 29.0 else pos[2] + 0.5
        
        self._engine.process_sensor_data(
            current_pos  = pos,
            current_vel  = vel,
            current_rpy  = rpy,
            target_pos   = target_3d,
            yaw_rate     = yaw_rate,
            agl          = safe_agl,
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

    def _setup_engine(self):
        """Configure and start the C engine."""
        cruise_alt = max(self._spawn_z + 3.0, 3.0)
        self._engine.set_flight_params(
            cruise_altitude      = cruise_alt,
            max_speed            = SPEED_LIMIT,
            safety_radius        = 1.15,
            mode_v               = 200.0,
            attraction_gain      = 15.0,
            landing_threshold_xy = 0.40,
            landing_threshold_z  = 1.50,
            landing_descent_rate = 0.025,
            platform_vel_est     = [0.0, 0.0, 0.0],
        )

        self._engine.start_adaptive()

# Copyright Nathanael J. Bocker, 2026 all rights reserved
