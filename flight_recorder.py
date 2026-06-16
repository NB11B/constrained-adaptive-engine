import csv
import os
from datetime import datetime

class FlightRecorder:
    def __init__(self, log_dir="flight_logs"):
        self.log_dir = log_dir
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        self.file = None
        self.writer = None
        self.headers = [
            "step", "time_s", 
            "pos_x", "pos_y", "pos_z",
            "vel_x", "vel_y", "vel_z",
            "agl", "dist_xy", "dist_3d",
            "target_x", "target_y", "target_z",
            "plat_vel_x", "plat_vel_y", "plat_vel_z",
            "cmd_vx", "cmd_vy", "cmd_vz", "cmd_speed", "cmd_yaw",
            "landing_phase", "descent_phase",
            "assist_type", "collision_risk", "clutter_density", "navigability",
            "escape_active", "pred_escape_active", "threat_acc",
            "terrain_id", "phase_reason", "rel_vx_to_pad", "rel_vy_to_pad"
        ]

    def start_trial(self, terrain_name, seed):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{terrain_name.replace('/', '_')}_s{seed}_{timestamp}.csv"
        path = os.path.join(self.log_dir, filename)
        self.file = open(path, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.headers)
        self.writer.writeheader()
        return path

    def record_step(self, step, time_s, current_pos, current_vel, agl, dist_xy, dist_3d, 
                    target_pos, plat_vel, control_output, cs, assist_type, terrain_id, phase_reason, rel_vx_to_pad, rel_vy_to_pad):
        if not self.writer:
            return
        
        row = {
            "step": step,
            "time_s": round(time_s, 3),
            "pos_x": round(current_pos[0], 4),
            "pos_y": round(current_pos[1], 4),
            "pos_z": round(current_pos[2], 4),
            "vel_x": round(current_vel[0], 4),
            "vel_y": round(current_vel[1], 4),
            "vel_z": round(current_vel[2], 4),
            "agl": round(agl, 3),
            "dist_xy": round(dist_xy, 3),
            "dist_3d": round(dist_3d, 3),
            "target_x": round(target_pos[0], 4),
            "target_y": round(target_pos[1], 4),
            "target_z": round(target_pos[2], 4),
            "plat_vel_x": round(plat_vel[0], 4),
            "plat_vel_y": round(plat_vel[1], 4),
            "plat_vel_z": round(plat_vel[2], 4),
            "cmd_vx": round(control_output[0], 4),
            "cmd_vy": round(control_output[1], 4),
            "cmd_vz": round(control_output[2], 4),
            "cmd_speed": round(control_output[3], 4),
            "cmd_yaw": round(control_output[4], 4),
            "landing_phase": int(cs.landing_phase),
            "descent_phase": int(cs.descent_phase),
            "assist_type": assist_type,
            "collision_risk": round(cs.collision_risk_score, 3),
            "clutter_density": round(cs.clutter_density, 3),
            "navigability": round(cs.navigability_score, 3),
            "escape_active": int(cs.escape_mode_active),
            "pred_escape_active": int(cs.predictive_escape_active),
            "threat_acc": round(cs.threat_accumulator, 3),
            "terrain_id": terrain_id,
            "phase_reason": phase_reason,
            "rel_vx_to_pad": round(rel_vx_to_pad, 4),
            "rel_vy_to_pad": round(rel_vy_to_pad, 4)
        }
        self.writer.writerow(row)

    def close(self):
        if self.file:
            self.file.close()
            self.file = None
            self.writer = None
