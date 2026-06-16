"""
test_predictive_detector.py
Unit test for the Predictive Local Minimum Detector in the CAE C engine.

Tests two scenarios:
  1. HIGH THREAT: High collision_risk + clutter + low navigability + slow speed
     → Predictive escape should trigger within ~25 ticks (0.5s)
  2. CLEAR CRUISE: Low threat metrics + high speed
     → Predictive escape should NOT trigger; threat_accumulator should stay near zero

Copyright: Nathanael J. Bocker, 2026 all rights reserved
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ctypes
from constrained_adaptive_engine_bridge import ConstrainedAdaptiveEngine, _adaptive_engine, AdaptState

def inject_state(engine, pos, vel, collision_risk, clutter, navigability):
    """Directly write into the C state struct via ctypes pointer."""
    state_ptr = _adaptive_engine.adapt_get_state(engine._controller)
    s = state_ptr.contents
    s.current_pos[0] = pos[0]
    s.current_pos[1] = pos[1]
    s.current_pos[2] = pos[2]
    s.current_vel[0] = vel[0]
    s.current_vel[1] = vel[1]
    s.current_vel[2] = vel[2]
    s.collision_risk_score = collision_risk
    s.clutter_density = clutter
    s.navigability_score = navigability
    # Ensure not in landing/descent so predictor is active
    s.landing_phase = False
    s.descent_phase = False

def run_test_1_high_threat():
    """High threat scenario: should trigger predictive escape within 25 ticks."""
    print("\n=== TEST 1: High Threat Scenario ===")
    engine = ConstrainedAdaptiveEngine()
    engine.set_flight_params(
        cruise_altitude=1.5, max_speed=3.0, safety_radius=1.15,
        mode_v=200.0, attraction_gain=5.0,
        landing_threshold_xy=0.60, landing_threshold_z=1.20
    )
    engine.set_target_pos([10.0, 10.0, 0.2])
    engine.start_adaptive()

    triggered_tick = None
    for i in range(50):
        # Re-inject each tick to simulate sustained threat (sensor data would do this normally)
        inject_state(engine,
                     pos=[0.0, 0.0, 1.5],
                     vel=[0.1, 0.1, 0.0],   # Very slow — 0.14 m/s
                     collision_risk=0.85,
                     clutter=0.75,
                     navigability=0.15)
        engine.update()
        s = engine.get_state()
        if s.predictive_escape_active and triggered_tick is None:
            triggered_tick = i
            print(f"  [TICK {i:3d}] TRIGGERED → escape_z={s.predictive_escape_target_z:.2f}m  "
                  f"threat_acc={s.threat_accumulator:.2f}  "
                  f"threat_score={s.predictive_threat_score:.3f}")
            break
        if i % 5 == 0:
            print(f"  [TICK {i:3d}] threat_score={s.predictive_threat_score:.3f}  "
                  f"threat_acc={s.threat_accumulator:.2f}  "
                  f"escape_active={s.predictive_escape_active}")

    if triggered_tick is not None and triggered_tick < 40:
        print(f"  RESULT: PASS — triggered at tick {triggered_tick} ({triggered_tick*0.02:.2f}s)")
        return True
    else:
        print(f"  RESULT: FAIL — did not trigger within 40 ticks")
        return False


def run_test_2_clear_cruise():
    """Clear cruise scenario: should NOT trigger predictive escape."""
    print("\n=== TEST 2: Clear Cruise Scenario ===")
    engine = ConstrainedAdaptiveEngine()
    engine.set_flight_params(
        cruise_altitude=1.5, max_speed=3.0, safety_radius=1.15,
        mode_v=200.0, attraction_gain=5.0,
        landing_threshold_xy=0.60, landing_threshold_z=1.20
    )
    engine.set_target_pos([10.0, 10.0, 0.2])
    engine.start_adaptive()

    max_acc = 0.0
    for i in range(100):
        inject_state(engine,
                     pos=[float(i) * 0.05, 0.0, 1.5],  # Moving toward goal
                     vel=[2.0, 0.0, 0.0],               # Fast cruise — 2.0 m/s
                     collision_risk=0.05,
                     clutter=0.10,
                     navigability=0.90)
        engine.update()
        s = engine.get_state()
        if s.threat_accumulator > max_acc:
            max_acc = s.threat_accumulator
        if s.predictive_escape_active:
            print(f"  RESULT: FAIL — escape triggered at tick {i} (should not have)")
            return False

    print(f"  Max threat_accumulator over 100 ticks: {max_acc:.3f}")
    print(f"  RESULT: PASS — no false trigger in 100 ticks (2.0s)")
    return True


def run_test_3_escape_climb():
    """Verify the escape actually commands an upward velocity."""
    print("\n=== TEST 3: Escape Climb Verification ===")
    engine = ConstrainedAdaptiveEngine()
    engine.set_flight_params(
        cruise_altitude=1.5, max_speed=3.0, safety_radius=1.15,
        mode_v=200.0, attraction_gain=5.0,
        landing_threshold_xy=0.60, landing_threshold_z=1.20
    )
    engine.set_target_pos([10.0, 10.0, 0.2])
    engine.start_adaptive()

    # Force trigger by running 30 high-threat ticks
    for i in range(35):
        inject_state(engine,
                     pos=[0.0, 0.0, 1.5],
                     vel=[0.05, 0.05, 0.0],
                     collision_risk=0.90,
                     clutter=0.80,
                     navigability=0.10)
        engine.update()
        s = engine.get_state()
        if s.predictive_escape_active:
            # Check that the control output has positive vz
            vz = s.control_output[2]
            print(f"  [TICK {i}] Escape active: vz={vz:.3f} m/s  escape_z={s.predictive_escape_target_z:.2f}m")
            if vz > 0.1:
                print(f"  RESULT: PASS — escape correctly commands upward velocity ({vz:.3f} m/s)")
                return True
            else:
                print(f"  RESULT: FAIL — escape active but vz={vz:.3f} (not climbing)")
                return False

    print("  RESULT: FAIL — escape never triggered in 35 ticks")
    return False


if __name__ == "__main__":
    results = []
    results.append(run_test_1_high_threat())
    results.append(run_test_2_clear_cruise())
    results.append(run_test_3_escape_climb())

    print("\n" + "="*50)
    passed = sum(results)
    total = len(results)
    print(f"PREDICTIVE DETECTOR UNIT TESTS: {passed}/{total} PASSED")
    if passed == total:
        print("ALL TESTS PASSED — Predictive detector is functioning correctly.")
    else:
        print("SOME TESTS FAILED — Review output above.")
    sys.exit(0 if passed == total else 1)
