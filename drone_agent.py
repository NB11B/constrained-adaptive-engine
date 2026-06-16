import numpy as np
from constrained_adaptive_engine_bridge import ConstrainedAdaptiveEngine

# ---------------------------------------------------------------------------
# Constants matching Swarm Subnet 124 observation/action contract
# ---------------------------------------------------------------------------
SPEED_LIMIT = 3.0        # m/s, Swarm validator velocity limit
DEPTH_MAX_RANGE = 20.0   # m, Swarm normalized depth map useful range: 0.5m-20m
DEPTH_WIDTH = 128
DEPTH_HEIGHT = 128
CAMERA_FOV_DEG = 90.0


# ---------------------------------------------------------------------------
# DroneFlightController (LITE VERSION)
# ---------------------------------------------------------------------------
class DroneFlightController:
    """
    Swarm Subnet 124 — Autonomous Drone Flight Controller (LITE)

    A streamlined, MCU-compliant controller using the AGL-based approach:
      1. Navigate to the noisy goal/search-area estimate from the state vector.
      2. Use AGL rangefinder changes to refine the pad XY estimate.
      3. Trigger the C-based CAE landing phase for precision touchdown.
    """

    def __init__(self):
        self._engine = ConstrainedAdaptiveEngine()
        self._debug = False
        self._reset_mission_state()

    def _reset_mission_state(self):
        self._initialized = False
        self._spawn_z = 1.8
        self._step = 0
        self._goal_xy = None
        self._goal_z = None

        # AGL calibration/search state. This must reset between benchmark seeds.
        self._calibrated_offset = np.zeros(3, dtype=np.float64)
        self._is_calibrated = False
        self._search_start_step = 0
        self._search_phase = 0  # 0: approach, 1: spiral search, 2: locked
        self._agl_history = []
        self._takeoff_complete = False
        self._last_agl = None

    def act(self, observation: dict) -> np.ndarray:
        """
        Compute flight action for the current observation.
        """
        state = observation["state"]
        depth = observation["depth"]

        # Unpack stable leading state-vector fields.
        pos = state[0:3].astype(np.float64)
        rpy = state[3:6].astype(np.float64)
        vel = state[6:9].astype(np.float64)
        ang_vel = state[9:12].astype(np.float64)
        yaw_rate = float(ang_vel[2])

        # Swarm appends altitude and search vector at the tail:
        # [..., altitude_norm, search_dx, search_dy, search_dz].
        # Tail indexing is resilient to action-history size changes.
        agl_norm = float(state[-4])
        agl_m = agl_norm * DEPTH_MAX_RANGE
        search_vec = state[-3:].astype(np.float64)

        # Sanitize AGL to suppress impossible one-frame range jumps.
        if self._last_agl is not None and abs(agl_m - self._last_agl) > 5.0:
            agl_m = self._last_agl
        self._last_agl = agl_m

        # Initialize engine on first call before using spawn-relative altitude logic.
        if not self._initialized:
            self._spawn_z = float(pos[2])
            self._setup_engine()
            self._initialized = True

        # 0. TAKEOFF PHASE
        if not self._takeoff_complete and pos[2] > (self._spawn_z + 2.5):
            self._takeoff_complete = True
            if self._debug:
                print("[TAKEOFF] Altitude reached. Starting mission.")

        # 1. NOISE CANCELLATION VIA AGL CALIBRATION
        noisy_center = pos + search_vec
        dist_to_noisy = np.linalg.norm(pos[:2] - noisy_center[:2])

        # Detect AGL dip: pad/landing surface is slightly raised relative to terrain.
        if not self._is_calibrated and dist_to_noisy < 5.0 and self._takeoff_complete:
            self._agl_history.append(agl_m)
            if len(self._agl_history) > 10:
                self._agl_history.pop(0)
            avg_agl = sum(self._agl_history) / len(self._agl_history)

            if agl_m < (avg_agl - 0.08) or agl_m < (pos[2] - 0.12):
                # If the AGL dip is detected while over the true pad, then:
                # true_pad_location = noisy_center + calibrated_offset
                self._calibrated_offset = np.array(
                    [pos[0] - noisy_center[0], pos[1] - noisy_center[1], 0.0],
                    dtype=np.float64,
                )
                self._is_calibrated = True
                if self._debug:
                    print(f"[CALIBRATION] Pad detected via AGL dip at step {self._step}.")

        # Apply calibration if found; otherwise use noisy center.
        true_target = noisy_center + self._calibrated_offset

        # If not calibrated, perform a search pattern around the noisy estimate.
        if not self._is_calibrated and dist_to_noisy < 5.0:
            if self._search_phase == 0:
                self._search_phase = 1
                self._search_start_step = self._step

            # Faster, tighter scan to cover the search uncertainty.
            t = (self._step - self._search_start_step) * 0.15
            r = 0.8 * (1 + (np.floor(t / 5.0) % 4))
            search_offset = np.array([r * np.cos(t), r * np.sin(t), 0.0], dtype=np.float64)
            true_target += search_offset
        elif self._is_calibrated and self._search_phase != 2:
            self._search_phase = 2
            if self._debug:
                print(f"[CALIBRATION] Search phase locked at step {self._step}.")

        # Always update C engine target position. Pad surface is nominally 0.2m.
        target_3d = np.array([true_target[0], true_target[1], 0.2], dtype=np.float64)
        self._engine.set_target_pos(target_3d)

        # Cruise altitude: lower for search, higher for cruise.
        search_alt = float(noisy_center[2]) + 1.2
        cruise_alt = max(self._spawn_z + 3.0, float(noisy_center[2]) + 2.5)
        in_search_zone = dist_to_noisy < 5.0

        if not self._is_calibrated:
            target_alt = search_alt if in_search_zone else cruise_alt
        else:
            # Once calibrated, descend to approach altitude to trigger C landing logic.
            target_alt = 1.0

        # Force takeoff altitude before mission cruise/search behavior.
        if not self._takeoff_complete:
            target_alt = self._spawn_z + 3.0

        # Terrain-following: never go below 1.0m AGL during uncalibrated search.
        if not self._is_calibrated and in_search_zone and self._takeoff_complete:
            target_alt = max(target_alt, pos[2] - agl_m + 1.0)

        self._engine.set_flight_params(
            cruise_altitude=target_alt,
            max_speed=SPEED_LIMIT,
            safety_radius=1.15,
            mode_v=200.0,
            attraction_gain=12.0,
            landing_threshold_xy=0.20,
            landing_threshold_z=1.50,
            landing_descent_rate=0.025,
            platform_vel_est=[0.0, 0.0, 0.0],
        )

        # Clamp AGL to prevent death dives on rangefinder voids.
        safe_agl = min(agl_m, pos[2] + 0.5) if agl_m < (DEPTH_MAX_RANGE - 1.0) else pos[2] + 0.5

        self._engine.process_sensor_data(
            current_pos=pos,
            current_vel=vel,
            current_rpy=rpy,
            target_pos=target_3d,
            yaw_rate=yaw_rate,
            agl=safe_agl,
            depth_image=depth,
            depth_width=DEPTH_WIDTH,
            depth_height=DEPTH_HEIGHT,
            max_range=DEPTH_MAX_RANGE,
            fov_deg=CAMERA_FOV_DEG,
        )

        self._engine.update()

        cs = self._engine.get_state()
        vx, vy, vz, total_speed, yaw_cmd = cs.control_output

        vel_mag = float(np.sqrt(vx**2 + vy**2 + vz**2)) + 1e-8
        dir_xyz = np.array([vx, vy, vz], dtype=np.float32) / vel_mag
        speed_norm = float(np.clip(total_speed / SPEED_LIMIT, 0.0, 1.0))

        # C engine already emits normalized yaw in [-1, 1]. Do not divide by pi again.
        yaw_norm = float(np.clip(yaw_cmd, -1.0, 1.0))

        action = np.array(
            [dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_norm],
            dtype=np.float32,
        )

        self._step += 1
        return action

    def reset(self):
        """Reset controller state at mission start."""
        self._engine.reset()
        self._reset_mission_state()

    def _setup_engine(self):
        """Configure and start the C engine."""
        cruise_alt = max(self._spawn_z + 3.0, 3.0)
        self._engine.set_flight_params(
            cruise_altitude=cruise_alt,
            max_speed=SPEED_LIMIT,
            safety_radius=1.15,
            mode_v=200.0,
            attraction_gain=15.0,
            landing_threshold_xy=0.40,
            landing_threshold_z=1.50,
            landing_descent_rate=0.025,
            platform_vel_est=[0.0, 0.0, 0.0],
        )
        self._engine.start_adaptive()

    def close(self):
        """Release native controller resources when supported by the bridge."""
        if hasattr(self._engine, "close"):
            self._engine.close()


# Copyright Nathanael J. Bocker, 2026 all rights reserved
