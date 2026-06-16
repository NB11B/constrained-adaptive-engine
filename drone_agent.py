"""
drone_agent.py — Swarm Subnet 142 Miner Submission
Constrained Adaptive Engine (CAE) integration for the Bittensor Swarm Subnet.

This file implements the DroneFlightController class required by the validator.
It bridges the PyBullet simulation environment with the C-based Constrained
Adaptive Engine (CAE) flight controller.

Architecture:
  - Observation: {"depth": (128,128,1) float32, "state": (141,) float32}
  - Action:      (5,) float32 [dir_x, dir_y, dir_z, speed_norm, yaw]
  - C Engine:    libadaptive_controller.so via ctypes bridge

State Vector Layout (141 dims, ctrl_freq=50, action_dim=4):
  [0:3]    = pos_xyz (meters)
  [3:6]    = rpy (radians)
  [6:9]    = vel_xyz (m/s)
  [9:12]   = ang_vel_xyz (rad/s)
  [12:112] = action_history (25 x 4)
  [112]    = AGL_norm (AGL / MAX_RAY_DISTANCE=20.0)
  [113:116]= search_area_center - drone_pos (meters, noisy ±10m XY)
  [116:141]= 25 additional state dims (reserved/extended)

Action Space (5-element, VelocityAviary):
  [0:3] = dir_xyz (unit direction vector, [-1,1])
  [3]   = speed_norm (fraction of SPEED_LIMIT=3.0 m/s, [0,1])
  [4]   = yaw (normalised to [-1,1], kept for validator compatibility)

Moving Platform Tracker:
  The _detect_pad() method uses the depth image to locate the flat circular
  landing pad near the noisy search_vec estimate. A Kalman-style EMA tracker
  maintains a smoothed position estimate and computes platform velocity for
  feed-forward compensation in the C engine.

Copyright: Nathanael J. Bocker, 2026 all rights reserved
"""

import os
import sys
import subprocess
import numpy as np

# ---------------------------------------------------------------------------
# Constants (must match swarm/constants.py)
# ---------------------------------------------------------------------------
SPEED_LIMIT       = 3.0    # m/s — from swarm.constants.SPEED_LIMIT
MAX_RAY_DISTANCE  = 20.0   # m  — from swarm.constants.MAX_RAY_DISTANCE
CAMERA_FOV_DEG    = 90.0   # degrees — CAMERA_FOV_BASE (±2 variance, use base)
DEPTH_WIDTH       = 128
DEPTH_HEIGHT      = 128

# Platform detection parameters
PAD_RADIUS_M      = 0.60   # m — landing pad radius
PAD_DETECT_WINDOW = 0.80   # m — search window around noisy estimate
VEL_EMA_ALPHA     = 0.25   # EMA smoothing for platform velocity estimate
POS_EMA_ALPHA     = 0.35   # EMA smoothing for platform position estimate

# ---------------------------------------------------------------------------
# Locate and compile the C library
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_LIB_PATH = os.path.join(_THIS_DIR, "libadaptive_controller.so")


def _ensure_library():
    """Compile the C library if it is missing or out of date."""
    src_files = [
        os.path.join(_THIS_DIR, "src", "adaptation_controller.c"),
        os.path.join(_THIS_DIR, "src", "psmsl_depth_processor.c"),
    ]
    include_dir = os.path.join(_THIS_DIR, "include")

    # Recompile if .so is missing or any source is newer
    needs_build = not os.path.exists(_LIB_PATH)
    if not needs_build:
        lib_mtime = os.path.getmtime(_LIB_PATH)
        needs_build = any(os.path.getmtime(s) > lib_mtime for s in src_files
                          if os.path.exists(s))

    if needs_build:
        cmd = [
            "gcc", "-O2", "-shared", "-fPIC",
            f"-I{include_dir}",
            "-o", _LIB_PATH,
        ] + src_files + ["-lm"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(
                f"CAE: Failed to compile C library:\n{result.stderr}"
            )


_ensure_library()

# Import bridge AFTER ensuring library exists
sys.path.insert(0, _THIS_DIR)
from constrained_adaptive_engine_bridge import ConstrainedAdaptiveEngine  # noqa: E402


# ---------------------------------------------------------------------------
# DroneFlightController
# ---------------------------------------------------------------------------
class DroneFlightController:
    """
    Swarm Subnet 142 — Autonomous Drone Flight Controller
    Powered by the Constrained Adaptive Engine (CAE).

    The controller uses a C-based potential-field navigation engine with:
      - Obstacle avoidance via PSMSL depth image analysis
      - Two-layer local minimum escape (predictive + reactive)
      - Stepped feathering landing profile (no overshoot)
      - Moving platform tracker with EMA velocity estimator
      - Velocity smoothing to prevent tilt truncation
    """

    def __init__(self):
        self._engine = ConstrainedAdaptiveEngine()
        self._initialized = False
        self._spawn_z = 1.8          # Default; overridden on first act() call
        self._goal = None            # Smoothed goal position estimate
        self._step = 0

        # Moving platform tracker state
        self._pad_pos_est = None     # EMA-smoothed pad position [x, y, z]
        self._pad_pos_prev = None    # Previous pad position for velocity calc
        self._pad_vel_est = np.zeros(3)  # EMA-smoothed platform velocity
        self._pad_detect_count = 0   # Consecutive successful detections
        self._last_search_vec = None # Previous search_vec for change detection

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def act(self, observation: dict) -> np.ndarray:
        """
        Compute flight action for the current observation.

        Args:
            observation: dict with:
                "depth" — (128, 128, 1) float32 normalized depth [0,1]
                "state" — (141,) float32 state vector

        Returns:
            numpy array (5,) — [dir_x, dir_y, dir_z, speed_norm, yaw]
        """
        state = observation["state"]
        depth = observation["depth"]

        # Unpack state vector
        pos      = state[0:3].astype(np.float64)
        rpy      = state[3:6].astype(np.float64)
        vel      = state[6:9].astype(np.float64)
        ang_vel  = state[9:12].astype(np.float64)
        yaw_rate = float(ang_vel[2])   # ang_vel[2] = yaw rate (rad/s)
        agl_norm = float(state[112])
        agl_m    = agl_norm * MAX_RAY_DISTANCE

        # Search area vector: target_center - drone_pos (noisy GPS, ±10m XY)
        search_vec = state[113:116].astype(np.float64)

        # Estimate goal position (raw noisy estimate)
        noisy_goal = pos + search_vec
        noisy_goal[2] = max(0.0, noisy_goal[2])

        # Run moving platform tracker
        goal = self._track_platform(pos, rpy, noisy_goal, depth)

        # Initialize engine on first call
        if not self._initialized:
            self._spawn_z = float(pos[2])
            self._goal = goal.copy()
            self._setup_engine(goal)
            self._initialized = True

        # Update smoothed goal
        self._goal = goal.copy()

        # Update platform velocity estimate in C engine params
        self._engine.set_flight_params(
            cruise_altitude      = self._spawn_z,
            max_speed            = SPEED_LIMIT,
            safety_radius        = 1.15,
            mode_v               = 200.0,
            attraction_gain      = 5.0,
            landing_threshold_xy = 0.60,
            landing_threshold_z  = 1.20,
            landing_descent_rate = 0.025,
            platform_vel_est     = self._pad_vel_est.tolist(),
        )

        # Feed sensor data to C engine
        self._engine.process_sensor_data(
            current_pos  = pos,
            current_vel  = vel,
            current_rpy  = rpy,
            target_pos   = goal,
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
        self._goal = None
        self._step = 0
        # Reset platform tracker
        self._pad_pos_est = None
        self._pad_pos_prev = None
        self._pad_vel_est = np.zeros(3)
        self._pad_detect_count = 0
        self._last_search_vec = None

    # ------------------------------------------------------------------
    # Moving Platform Tracker
    # ------------------------------------------------------------------

    def _track_platform(self,
                        pos: np.ndarray,
                        rpy: np.ndarray,
                        noisy_goal: np.ndarray,
                        depth: np.ndarray) -> np.ndarray:
        """
        Track the landing pad position and estimate its velocity.

        Strategy:
          1. Attempt to detect the pad in the depth image near the noisy goal.
          2. If detected, update the EMA position estimate and compute velocity.
          3. If not detected, fall back to the noisy goal with decaying velocity.

        Returns the best available goal estimate as a (3,) float64 array.
        """
        # Attempt pad detection in depth image
        detected_pos = self._detect_pad(pos, rpy, noisy_goal, depth)

        if detected_pos is not None:
            self._pad_detect_count += 1

            if self._pad_pos_est is None:
                # First detection — initialize tracker
                self._pad_pos_est = detected_pos.copy()
                self._pad_pos_prev = detected_pos.copy()
            else:
                # Update EMA position estimate
                self._pad_pos_prev = self._pad_pos_est.copy()
                self._pad_pos_est = (
                    (1.0 - POS_EMA_ALPHA) * self._pad_pos_est
                    + POS_EMA_ALPHA * detected_pos
                )

                # Compute instantaneous velocity (m/s at 50Hz → multiply by 50)
                # Only update velocity if we have at least 5 consecutive detections
                if self._pad_detect_count >= 5:
                    raw_vel = (self._pad_pos_est - self._pad_pos_prev) * 50.0
                    # Clamp to physically plausible platform speed (≤2 m/s)
                    raw_vel = np.clip(raw_vel, -2.0, 2.0)
                    self._pad_vel_est = (
                        (1.0 - VEL_EMA_ALPHA) * self._pad_vel_est
                        + VEL_EMA_ALPHA * raw_vel
                    )
        else:
            # Detection failed — decay velocity estimate toward zero
            self._pad_detect_count = max(0, self._pad_detect_count - 1)
            self._pad_vel_est *= 0.95  # Exponential decay

        # Return best available goal estimate
        if self._pad_pos_est is not None:
            return self._pad_pos_est.copy()
        return noisy_goal.copy()

    def _detect_pad(self,
                    pos: np.ndarray,
                    rpy: np.ndarray,
                    noisy_goal: np.ndarray,
                    depth: np.ndarray) -> np.ndarray | None:
        """
        Detect the flat circular landing pad in the depth image.

        The pad is a flat disc of radius ~0.6m. We unproject depth pixels to
        3D world coordinates and look for a cluster of flat points near the
        noisy goal estimate. Returns the centroid of the detected pad, or None.

        This is a fast NumPy-only implementation (no scipy/sklearn) suitable
        for 50Hz execution.
        """
        # Only run detection when we are within plausible detection range
        dist_to_goal = float(np.linalg.norm(noisy_goal[:2] - pos[:2]))
        if dist_to_goal > 25.0:
            return None  # Too far — noisy estimate unreliable

        # Unproject depth image to 3D world points
        world_pts = self._unproject_depth(pos, rpy, depth)
        if world_pts is None or len(world_pts) == 0:
            return None

        # Filter points near the noisy goal XY position
        dx = world_pts[:, 0] - noisy_goal[0]
        dy = world_pts[:, 1] - noisy_goal[1]
        xy_dist = np.sqrt(dx**2 + dy**2)
        near_mask = xy_dist < (PAD_RADIUS_M + PAD_DETECT_WINDOW)
        near_pts = world_pts[near_mask]

        if len(near_pts) < 8:
            return None  # Not enough points for a reliable detection

        # Find the flattest cluster (low Z variance = flat surface = pad)
        z_vals = near_pts[:, 2]
        z_median = float(np.median(z_vals))
        flat_mask = np.abs(z_vals - z_median) < 0.15  # ±15cm flatness threshold
        flat_pts = near_pts[flat_mask]

        if len(flat_pts) < 5:
            return None

        # Centroid of flat cluster = pad center estimate
        centroid = np.mean(flat_pts, axis=0)

        # Sanity check: centroid must be within 3m of noisy goal
        centroid_dist = float(np.linalg.norm(centroid[:2] - noisy_goal[:2]))
        if centroid_dist > 3.0:
            return None

        # Use the noisy goal Z if centroid Z is implausible (below ground)
        if centroid[2] < 0.0:
            centroid[2] = max(0.0, noisy_goal[2])

        return centroid.astype(np.float64)

    def _unproject_depth(self,
                         pos: np.ndarray,
                         rpy: np.ndarray,
                         depth: np.ndarray) -> np.ndarray | None:
        """
        Unproject a depth image to 3D world coordinates using NumPy.

        Uses a pinhole camera model with the drone's pose. Subsamples the
        image to a 16×16 grid for 50Hz performance.

        Returns an (N, 3) float64 array of world-frame 3D points, or None.
        """
        depth_2d = depth[:, :, 0] if depth.ndim == 3 else depth
        H, W = depth_2d.shape

        # Subsample to 16×16 for speed
        step_y = max(1, H // 16)
        step_x = max(1, W // 16)
        depth_sub = depth_2d[::step_y, ::step_x]
        sh, sw = depth_sub.shape

        # Build pixel grid
        ys = np.arange(sh) * step_y + step_y // 2
        xs = np.arange(sw) * step_x + step_x // 2
        xv, yv = np.meshgrid(xs, ys)
        xv = xv.flatten().astype(np.float64)
        yv = yv.flatten().astype(np.float64)
        d_vals = depth_sub.flatten().astype(np.float64) * MAX_RAY_DISTANCE

        # Filter out invalid/background pixels
        valid = (d_vals > 0.1) & (d_vals < MAX_RAY_DISTANCE * 0.98)
        if not np.any(valid):
            return None
        xv, yv, d_vals = xv[valid], yv[valid], d_vals[valid]

        # Pinhole camera model
        fov_rad = np.deg2rad(CAMERA_FOV_DEG)
        fx = (W / 2.0) / np.tan(fov_rad / 2.0)
        fy = fx  # Square pixels assumed

        # Camera-frame rays (x right, y down, z forward)
        cx_cam = (xv - W / 2.0) / fx * d_vals
        cy_cam = (yv - H / 2.0) / fy * d_vals
        cz_cam = d_vals

        # Rotate from camera frame to world frame using drone RPY
        roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])

        # Rotation matrices (ZYX convention)
        cr, sr = np.cos(roll),  np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw),   np.sin(yaw)

        # R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        R = np.array([
            [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
            [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
            [-sp,    cp*sr,             cp*cr            ],
        ])

        # Camera points stacked as (3, N)
        cam_pts = np.vstack([cx_cam, cy_cam, cz_cam])  # (3, N)

        # World points = drone_pos + R @ cam_pts
        world_pts = pos[:, np.newaxis] + R @ cam_pts  # (3, N)

        return world_pts.T  # (N, 3)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _setup_engine(self, goal: np.ndarray):
        """Configure and start the C engine for a new episode."""
        self._engine.set_flight_params(
            cruise_altitude      = self._spawn_z,  # Match spawn height
            max_speed            = SPEED_LIMIT,    # 3.0 m/s
            safety_radius        = 1.15,
            mode_v               = 200.0,
            attraction_gain      = 5.0,
            landing_threshold_xy = 0.60,
            landing_threshold_z  = 1.20,
            landing_descent_rate = 0.025,
            platform_vel_est     = [0.0, 0.0, 0.0],
        )
        self._engine.set_target_pos(goal)
        self._engine.start_adaptive()
