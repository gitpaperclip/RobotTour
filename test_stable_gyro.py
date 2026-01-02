"""
test_stable_gyro.py - Test script for StableGyro drift performance

This script simulates a typical Science Olympiad Robot Tour mission:
- Multiple forward movements
- 90° and 180° turns
- 1-2 minute total duration

Use this to validate that:
1. Heading remains stable over the mission
2. ZUPT corrections are working
3. Turns are accurate
4. Return to starting heading is precise
"""

import StableGyro as sg
import time

def test_drift_stationary(duration_seconds=120):
    """
    Test 1: Measure drift while completely stationary.
    This validates ZUPT is working - drift should be minimal.

    Args:
        duration_seconds: How long to measure (default 2 minutes)
    """
    print("\n" + "="*60)
    print("TEST 1: Stationary Drift Test")
    print("="*60)
    print(f"Robot will remain stationary for {duration_seconds} seconds")
    print("Measuring heading drift with ZUPT enabled...")
    print("DO NOT MOVE THE ROBOT!")
    print()

    # Reset heading
    sg.reset_heading(0.0)
    start_heading = sg.get_heading()
    start_bias = sg.get_bias()

    # Wait and periodically check
    start_time = time.ticks_ms()
    samples = []

    print(f"{'Time (s)':<10} {'Heading':<12} {'Drift':<12} {'Bias':<12}")
    print("-" * 60)

    while time.ticks_diff(time.ticks_ms(), start_time) < duration_seconds * 1000:
        elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000.0
        current_heading = sg.get_heading()
        current_bias = sg.get_bias()
        drift = current_heading - start_heading

        # Log every 10 seconds
        if len(samples) == 0 or elapsed - samples[-1][0] >= 10:
            samples.append((elapsed, current_heading, drift, current_bias))
            print(f"{elapsed:<10.1f} {current_heading:<12.4f} {drift:<12.4f} {current_bias:<12.4f}")

        time.sleep_ms(1000)

    # Final results
    final_heading = sg.get_heading()
    final_bias = sg.get_bias()
    total_drift = final_heading - start_heading
    bias_change = final_bias - start_bias

    print("\n" + "-" * 60)
    print("RESULTS:")
    print(f"  Duration: {duration_seconds} seconds")
    print(f"  Starting heading: {start_heading:.4f}°")
    print(f"  Final heading: {final_heading:.4f}°")
    print(f"  Total drift: {total_drift:.4f}°")
    print(f"  Drift rate: {total_drift/duration_seconds:.4f}°/s")
    print(f"  Starting bias: {start_bias:.4f} dps")
    print(f"  Final bias: {final_bias:.4f} dps")
    print(f"  Bias change: {bias_change:.4f} dps")

    # ZUPT stats
    stats = sg.get_zupt_stats()
    print(f"\nZUPT Statistics:")
    print(f"  Corrections applied: {stats['correction_count']}")
    print(f"  Total correction: {stats['total_correction']:.4f} dps")
    if stats['correction_count'] > 0:
        print(f"  Avg correction: {stats['avg_correction']:.4f} dps")

    print("="*60 + "\n")

    return total_drift


def test_square_pattern():
    """
    Test 2: Drive a square pattern and return to start.
    Validates turn accuracy and straight-line stability.
    """
    print("\n" + "="*60)
    print("TEST 2: Square Pattern Test")
    print("="*60)
    print("Robot will drive a 30cm square")
    print("After 4x 90° turns, should return to original heading")
    print()

    sg.reset_heading(0.0)
    start_heading = sg.get_heading()

    side_length = 30  # cm
    turn_angle = 90  # degrees

    print(f"Starting heading: {start_heading:.2f}°\n")

    for i in range(4):
        print(f"Side {i+1}/4:")

        # Drive forward
        print(f"  Driving {side_length} cm forward...")
        sg.drive_straight(side_length, base_speed=600)
        time.sleep_ms(500)

        # Turn 90 degrees
        target = (start_heading + (i+1) * turn_angle) % 360
        print(f"  Turning to {target:.1f}°...")
        sg.turn_to_angle(target, base_speed=800, tolerance=1.5, validate_with_encoders=True)
        time.sleep_ms(500)

        current = sg.get_heading()
        error = current - target
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        print(f"  Heading: {current:.2f}° (error: {error:.2f}°)\n")

    # Final heading should be close to start
    final_heading = sg.get_heading()
    total_error = final_heading - start_heading
    if total_error > 180:
        total_error -= 360
    elif total_error < -180:
        total_error += 360

    print("-" * 60)
    print("RESULTS:")
    print(f"  Starting heading: {start_heading:.2f}°")
    print(f"  Final heading: {final_heading:.2f}°")
    print(f"  Cumulative error: {total_error:.2f}°")
    print(f"  Error per turn: {total_error/4:.2f}°")

    sg.print_diagnostics()

    return abs(total_error)


def test_typical_mission():
    """
    Test 3: Simulate a typical Science Olympiad mission.
    Mix of forward/backward movements and various turns.
    """
    print("\n" + "="*60)
    print("TEST 3: Typical Mission Simulation")
    print("="*60)
    print("Simulating a 90-second mission with various moves")
    print()

    sg.reset_heading(0.0)
    start_heading = sg.get_heading()
    mission_start = time.ticks_ms()

    # Mission sequence
    moves = [
        ("Forward 40cm", lambda: sg.drive_straight(40, base_speed=700)),
        ("Turn 90° right", lambda: sg.turn_relative(90, base_speed=800)),
        ("Forward 30cm", lambda: sg.drive_straight(30, base_speed=650)),
        ("Turn 180°", lambda: sg.turn_relative(180, base_speed=750)),
        ("Backward 25cm", lambda: sg.drive_straight(-25, base_speed=600)),
        ("Turn 90° left", lambda: sg.turn_relative(-90, base_speed=800)),
        ("Forward 35cm", lambda: sg.drive_straight(35, base_speed=700)),
        ("Turn back to start", lambda: sg.turn_to_angle(start_heading, base_speed=800)),
    ]

    for i, (description, move_func) in enumerate(moves):
        print(f"\nMove {i+1}/{len(moves)}: {description}")
        move_func()

        # Pause between moves
        time.sleep_ms(1000)

        # Show status
        current = sg.get_heading()
        elapsed = time.ticks_diff(time.ticks_ms(), mission_start) / 1000.0
        print(f"  Current heading: {current:.2f}°")
        print(f"  Mission time: {elapsed:.1f}s")

    # Final results
    mission_duration = time.ticks_diff(time.ticks_ms(), mission_start) / 1000.0
    final_heading = sg.get_heading()
    heading_error = final_heading - start_heading
    if heading_error > 180:
        heading_error -= 360
    elif heading_error < -180:
        heading_error += 360

    print("\n" + "-" * 60)
    print("MISSION COMPLETE!")
    print(f"  Duration: {mission_duration:.1f} seconds")
    print(f"  Starting heading: {start_heading:.2f}°")
    print(f"  Final heading: {final_heading:.2f}°")
    print(f"  Return-to-start error: {heading_error:.2f}°")

    sg.print_diagnostics()

    return abs(heading_error)


def test_turn_accuracy():
    """
    Test 4: Measure accuracy of 90° and 180° turns.
    """
    print("\n" + "="*60)
    print("TEST 4: Turn Accuracy Test")
    print("="*60)
    print("Testing precision of 90° and 180° turns")
    print()

    # Reset to 0
    sg.reset_heading(0.0)
    time.sleep_ms(500)

    test_angles = [90, 180, 270, 0, 45, 135, 225, 315, 0]
    errors = []

    print(f"{'Target':<10} {'Actual':<10} {'Error':<10}")
    print("-" * 35)

    for target in test_angles:
        sg.turn_to_angle(target, base_speed=800, tolerance=1.0, validate_with_encoders=True)
        time.sleep_ms(1000)  # Settle time

        actual = sg.get_heading()
        error = actual - target
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        errors.append(error)
        print(f"{target:<10} {actual:<10.2f} {error:<10.2f}")

    # Statistics
    avg_error = sum(errors) / len(errors)
    abs_errors = [abs(e) for e in errors]
    avg_abs_error = sum(abs_errors) / len(abs_errors)
    max_error = max(abs_errors)

    print("\n" + "-" * 35)
    print("STATISTICS:")
    print(f"  Average error: {avg_error:.2f}°")
    print(f"  Average absolute error: {avg_abs_error:.2f}°")
    print(f"  Maximum error: {max_error:.2f}°")
    print("="*60 + "\n")

    return avg_abs_error


def run_all_tests():
    """Run complete test suite."""
    print("\n" + "="*60)
    print("STABLE GYRO - COMPLETE TEST SUITE")
    print("="*60)

    # Initialize
    print("\nInitializing system...")
    sg.initialize()

    # Wait for user
    sg.display.fill(0)
    sg.display.text("Tests Ready", 0, 20)
    sg.display.text("Press A", 0, 35)
    sg.display.show()

    # Note: You'll need to add button handling or just remove this if testing via upload
    time.sleep_ms(3000)

    results = {}

    # Test 1: Stationary drift (30 seconds for quick test)
    results['stationary_drift'] = test_drift_stationary(duration_seconds=30)
    time.sleep_ms(2000)

    # Test 2: Square pattern
    results['square_error'] = test_square_pattern()
    time.sleep_ms(2000)

    # Test 3: Turn accuracy
    results['turn_accuracy'] = test_turn_accuracy()
    time.sleep_ms(2000)

    # Test 4: Typical mission
    results['mission_error'] = test_typical_mission()

    # Summary
    print("\n" + "="*60)
    print("FINAL SUMMARY")
    print("="*60)
    print(f"Stationary drift (30s): {results['stationary_drift']:.2f}°")
    print(f"Square pattern error: {results['square_error']:.2f}°")
    print(f"Turn accuracy (avg): {results['turn_accuracy']:.2f}°")
    print(f"Mission return error: {results['mission_error']:.2f}°")

    sg.print_diagnostics()

    print("\nAll tests complete!")
    print("="*60 + "\n")


# ============================================================================
# SIMPLE DEMO (for quick testing)
# ============================================================================

def simple_demo():
    """
    Simple demonstration of StableGyro features.
    Good for quick validation after uploading to robot.
    """
    print("\nStableGyro - Simple Demo")
    print("="*40)

    # Initialize
    sg.initialize()

    print("\n1. Driving forward 30 cm...")
    sg.drive_straight(30, base_speed=650)
    time.sleep_ms(1000)

    print("\n2. Turning 90° right...")
    sg.turn_relative(90, base_speed=800)
    time.sleep_ms(1000)

    print("\n3. Driving forward 30 cm...")
    sg.drive_straight(30, base_speed=650)
    time.sleep_ms(1000)

    print("\n4. Turning back to start (90° left)...")
    sg.turn_relative(-90, base_speed=800)
    time.sleep_ms(1000)

    print("\n5. Returning to start...")
    sg.drive_straight(-30, base_speed=600)

    print("\nDemo complete!")
    sg.print_diagnostics()


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    # Choose what to run:

    # Option 1: Simple quick demo
    simple_demo()

    # Option 2: Run full test suite
    # run_all_tests()

    # Option 3: Individual tests
    # sg.initialize()
    # test_drift_stationary(duration_seconds=60)
    # test_square_pattern()
    # test_turn_accuracy()
    # test_typical_mission()
