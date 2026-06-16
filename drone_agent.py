import os
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

# Landing/search policy constants
SEARCH_ZONE_M = 5.0
COMMIT_ZONE_M = 1.35
LOW_PASS_AGL_M = 1.80
SEARCH_DWELL_TICKS = 180       # 3.6s at 50Hz inside the search zone
COMMIT_DWELL_TICKS = 35        # 0.7s near target before forced commit
TERMINAL_ASSIST_XY_M = 1.25
TERMINAL_ASSIST_AGL_M = 2.20


# ---------------------------------------------------------------------------
# DroneFlightController (LITE VERSION)
# ---------------------------------------------------------------------------
class DroneFlightController:
    """
    Swarm Subnet 124 — Autonomous Drone Flight Controller (LITE)

    A streamlined, MCU-compliant controller using the AGL-based approach:
      1. Navigate to the noisy goal/search-area estimate from the state vector.
      2. Use AGL rangefinder changes and search-zone dwell to refine/commit.
      3. Trigger the C-based CAE landing phase for precision touchdown.
    """

    def __init__(self):
        self._engine = ConstrainedAdaptiveEngine()
        self._debug = os.environ.get("CAE_DEBUG", "0") == "1"
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
        self._landing_committed = False
        self._commit_reason = "none"
        self._search_start_step = 0
        self._search_phase = 0  # 0: approach, 1: active sweep, 2: committed/locked
        self._agl_history = []
        self._takeoff_complete = False
        self._last_agl = None
        self._near_target_ticks = 0
        self._search_zone_ticks = 0
        self._last_debug_step = -9999

    def _commit_landing(self, reason: str, pos: np.ndarray, noisy_center: np.ndarray, use_overflight_offset: bool = False):
        """Commit to the current best target estimate and hand final descent to C."""
        if not self._landing_committed:
            if not self._is_calibrated:
                if use_overflight_offset:
                    # Only use current-position offset when AGL/low-pass evidence says the drone is actually over the pad.
                    self._calibrated_offset = np.array(
                        [pos[0] - noisy_center[0], pos[1] - noisy_center[1], 0.0],
                        dtype=np.float64,
                    )
                else:
                    # Fallback commit should land on the best available Swarm reference, not on the drone's current XY.
                    self._calibrated_offset = np.zeros(3, dtype=np.float64)
                self._is_calibrated = True
            self._landing_committed = True
            self._commit_reason = reason
            self._search_phase = 2
            if self._debug:
                print(f"[LANDING-COMMIT] {reason} at step {self._step}")

    def _terminal_assist_action(self, pos, target_3d, yaw_norm):
        """Direct touchdown assist when committed and close enough that the C field should stop searching."""
        delta = target_3d - pos
        xy_dist = float(np.linalg.norm(delta[:2]))
        h_rem = max(float(pos[2] - target_3d[2]), 0.0)

        if xy_dist < 0.18:
            desired = np.array([0.0, 0.0, -0.55 if h_rem > 0.35 else -0.28], dtype=np.float32)
        else:
            xy_speed = min(0.65, max(0.12, xy_dist * 0.85))
            desired = np.array(
                [delta[0] / (xy_dist + 1e-8) * xy_speed,
                 delta[1] / (xy_dist + 1e-8) * xy_speed,
                 -0.55 if h_rem > 0.35 else -0.25],
                dtype=np.float32,
            )

        vel_mag = float(np.linalg.norm(desired)) + 1e-8
        dir_xyz = desired / vel_mag
        speed_norm = float(np.clip(vel_mag / SPEED_LIMIT, 0.0, 1.0))
        return np.array([dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_norm], dtype=np.float32)

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

        noisy_center = pos + search_vec
        dist_to_noisy = np.linalg.norm(pos[:2] - noisy_center[:2])

        if self._takeoff_complete and dist_to_noisy < SEARCH_ZONE_M:
            self._search_zone_ticks += 1
        else:
            self._search_zone_ticks = 0

        if dist_to_noisy < COMMIT_ZONE_M:
            self._near_target_ticks += 1
        else:
            self._near_target_ticks = 0

        # 1. NOISE CANCELLATION VIA AGL CALIBRATION
        # Keep the original AGL-dip path, but do not make it the only way to land.
        if not self._is_calibrated and dist_to_noisy < SEARCH_ZONE_M and self._takeoff_complete:
            self._agl_history.append(agl_m)
            if len(self._agl_history) > 20:
                self._agl_history.pop(0)
            avg_agl = sum(self._agl_history) / len(self._agl_history)
            min_recent_agl = min(self._agl_history)

            agl_dip_detected = (
                agl_m < (avg_agl - 0.06)
                or (agl_m <= (min_recent_agl + 0.03) and len(self._agl_history) >= 10)
                or (agl_m < LOW_PASS_AGL_M and dist_to_noisy < COMMIT_ZONE_M)
            )
            if agl_dip_detected:
                self._calibrated_offset = np.array(
                    [pos[0] - noisy_center[0], pos[1] - noisy_center[1], 0.0],
                    dtype=np.float64,
                )
                self._is_calibrated = True
                if self._debug:
                    print(f"[CALIBRATION] Pad/low-pass detected at step {self._step}.")

        # Commit if we have repeatedly crossed the expected pad region, even without a perfect AGL dip.
        if self._takeoff_complete and not self._landing_committed:
            if self._is_calibrated and dist_to_noisy < 2.25:
                self._commit_landing("calibrated target proximity", pos, noisy_center, use_overflight_offset=False)
            elif self._near_target_ticks >= COMMIT_DWELL_TICKS:
                self._commit_landing("near-target dwell", pos, noisy_center, use_overflight_offset=False)
            elif self._search_zone_ticks >= SEARCH_DWELL_TICKS and agl_m < 2.4:
                self._commit_landing("search-zone low-altitude dwell", pos, noisy_center, use_overflight_offset=False)

        # Apply calibration if found; otherwise use noisy center.
        true_target = noisy_center + self._calibrated_offset

        # If not committed, perform a deterministic cross/square sweep around the noisy estimate.
        if not self._landing_committed and dist_to_noisy < SEARCH_ZONE_M:
            if self._search_phase == 0:
                self._search_phase = 1
                self._search_start_step = self._step

            phase_tick = self._step - self._search_start_step
            leg = (phase_tick // 45) % 4
            amp = min(2.4, 0.7 + 0.18 * (phase_tick // 180))
            if leg == 0:
                search_offset = np.array([amp, 0.0, 0.0], dtype=np.float64)
            elif leg == 1:
                search_offset = np.array([0.0, amp, 0.0], dtype=np.float64)
            elif leg == 2:
                search_offset = np.array([-amp, 0.0, 0.0], dtype=np.float64)
            else:
                search_offset = np.array([0.0, -amp, 0.0], dtype=np.float64)
            true_target += search_offset

        # Always update C engine target position. Pad surface is nominally 0.2m.
        target_3d = np.array([true_target[0], true_target[1], 0.2], dtype=np.float64)
        self._engine.set_target_pos(target_3d)

        # Cruise altitude: lower for active search, lower still once committed.
        in_search_zone = dist_to_noisy < SEARCH_ZONE_M
        search_alt = float(noisy_center[2]) + 0.95
        cruise_alt = max(self._spawn_z + 3.0, float(noisy_center[2]) + 2.5)

        if self._landing_committed:
            target_alt = 0.55
        elif in_search_zone:
            target_alt = search_alt
        else:
            target_alt = cruise_alt

        # Force takeoff altitude before mission cruise/search behavior.
        if not self._takeoff_complete:
            target_alt = self._spawn_z + 3.0

        # Terrain-following: keep enough AGL during uncommitted search, but allow low passes.
        if not self._landing_committed and in_search_zone and self._takeoff_complete:
            target_alt = max(target_alt, pos[2] - agl_m + 0.85)

        self._engine.set_flight_params(
            cruise_altitude=target_alt,
            max_speed=SPEED_LIMIT,
            safety_radius=0.70 if self._landing_committed else 1.10,
            mode_v=55.0 if self._landing_committed else 160.0,
            attraction_gain=15.0 if self._landing_committed else 11.0,
            landing_threshold_xy=0.85 if self._landing_committed else 0.65,
            landing_threshold_z=1.20,
            landing_descent_rate=0.58 if self._landing_committed else 0.45,
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
        yaw_norm = float(np.clip(yaw_cmd, -1.0, 1.0))

        action = np.array(
            [dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_norm],
            dtype=np.float32,
        )

        target_xy_dist = float(np.linalg.norm(target_3d[:2] - pos[:2]))
        if self._landing_committed and target_xy_dist < TERMINAL_ASSIST_XY_M and agl_m < TERMINAL_ASSIST_AGL_M:
            action = self._terminal_assist_action(pos, target_3d, yaw_norm)

        if self._debug and self._step - self._last_debug_step >= 50:
            self._last_debug_step = self._step
            print(
                f"[CAE] step={self._step} pos={pos.round(2)} agl={agl_m:.2f} "
                f"d_noisy={dist_to_noisy:.2f} d_tgt={target_xy_dist:.2f} "
                f"commit={self._landing_committed} reason={self._commit_reason} "
                f"landing={bool(cs.landing_phase)} descent={bool(cs.descent_phase)} "
                f"near_ticks={self._near_target_ticks} search_ticks={self._search_zone_ticks} "
                f"action={action.round(3)}"
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
            safety_radius=1.10,
            mode_v=160.0,
            attraction_gain=11.0,
            landing_threshold_xy=0.65,
            landing_threshold_z=1.20,
            landing_descent_rate=0.45,
            platform_vel_est=[0.0, 0.0, 0.0],
        )
        self._engine.start_adaptive()

    def close(self):
        """Release native controller resources when supported by the bridge."""
        if hasattr(self._engine, "close"):
            self._engine.close()


# Copyright Nathanael J. Bocker, 2026 all rights reserved
