# Code Connection Knowledge Graph

Generated: `2026-06-20T18:09:17.174646`
Source: `/mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE/code_connection_kg_v1/candidate/runtime/cae_agent_impl.py`

## Critical execution path

`observation → _extract_state → _estimate_target → target_pos/dist_xy → goal_corrections → _infer_terrain → _landing_params → engine.set_target_pos/process_sensor_data/update → control_output → _action_from_velocity → action`

## Extracted goal correction blocks

| Name | Line | Init range | Raw-search range | Step min | Offset | z_floor |
|---|---:|---:|---:|---:|---|---:|
| city | 515 | (-1.0, 8.5] | (-1.0, 3.5] | 0 | `np.array([-0.25, -1.42, -1.02], dtype=np.float64)` | None |
| open | 525 | (14.0, 19.5] | (-1.0, 9.0] | 250 | `np.array([1.73, -6.25, 0.20], dtype=np.float64)` | 0.2 |
| forest | 535 | (18.0, 30.0] | (-1.0, 8.5] | 250 | `np.array([6.51, -3.53, 1.63], dtype=np.float64)` | 1.83 |
| mountain | 545 | (40.0, 50.0] | (20.0, 999.0] | 250 | `np.array([0.0, 0.0, 2.0], dtype=np.float64)` | 2.2 |
| village | 558 | (40.0, 50.0] | (10.0, 20.0] | 250 | `np.array([0.0, 0.0, 0.0], dtype=np.float64)` | None |

## Disconnect warnings

| Severity | Code | Finding | Evidence |
|---|---|---|---|
| high | `VILLAGE_GAIN_WITH_NEUTRAL_VILLAGE_BLOCK` | The candidate comments say Village is neutral, but the confirmed score improvement is on Village. | village correction line 558 offset=np.array([0.0, 0.0, 0.0], dtype=np.float64) z_floor=None |
| high | `MOUNTAIN_CORRECTION_NOT_ACTUAL_PLATFORM` | Mountain correction only adds z +2 / z_floor 2.20, but the adjusted Mountain platform is around z=28.10. | mountain correction line 545 offset=np.array([0.0, 0.0, 2.0], dtype=np.float64) z_floor=2.2 |
| medium | `MOUNTAIN_CLASSIFIER_START_Z_GAP` | Candidate v1 classifier still uses spawn_z > 18.0, while adjusted Mountain start is z≈10.72. | terrain classifier line contains `if self.spawn_z > 18.0 or agl > 7.0:` |
| medium | `TARGET_ESTIMATE_Z_CLAMP` | Initial target estimate clamps z near ground, so Mountain's high adjusted platform altitude cannot be inferred through _estimate_target. | lines near _estimate_target clamp raw_target[2] to 0.20 or >=0.10 |
| medium | `LOCAL_TARGET_POS_NOT_PERSISTED_TO_TARGET_EST` | Goal corrections overwrite local target_pos but do not persist it into self.target_est; correction must re-trigger every step. | target_pos is sent to engine, but self.target_est is not updated in the correction block |
| low | `GOAL_CORRECTION_PREDICATE_OVERLAP` | Correction predicates overlap: open and forest. First match wins. | open L525 vs forest L535 |
| low | `GOAL_CORRECTION_PREDICATE_OVERLAP` | Correction predicates overlap: forest and open. First match wins. | forest L535 vs open L525 |

## Outputs

- JSON: `/mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE/code_connection_kg_v1/code_flow_kg.json`
- DOT: `/mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE/code_connection_kg_v1/code_flow_kg.dot`
- Disconnects TSV: `/mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE/code_connection_kg_v1/disconnects.tsv`

Render with:

```bash
dot -Tsvg /mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE/code_connection_kg_v1/code_flow_kg.dot -o /mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE/code_connection_kg_v1/code_flow_kg.svg
```
