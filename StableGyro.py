"""
StableGyro.py - Unified Robot Control with Drift-Resistant Heading Estimation

Implements dynamic bias tracking to maintain stable yaw estimates over 1-2 minute missions.
Uses ZUPT (Zero-Velocity Updates) and encoder validation to correct drift in real-time.

Key Features:
- Multi-stage gyro calibration (cold + warm)
- ZUPT: Corrects bias during stationary periods
- Encoder cross-validation during turns
- Thread-safe gyro integration
- Clean motion control API
"""

from pololu_3pi_2040_robot import robot
import time
import _thread
import math

# Hardware setup
motors = robot.Motors()
encoders = robot.Encoders()
display = robot.Display()
imu = robot.IMU()
imu.reset()
imu.enable_default()

# ============================================================================
# CONFIGURATION
# ============================================================================

# Robot physical parameters
TICKS_PER_ROTATION = 12
GEAR_RATIO = 29.86
WHEEL_CIRCUMFERENCE_LEFT = 10.15  # cm
WHEEL_CIRCUMFERENCE_RIGHT = 10.40  # cm
WHEEL_BASE = 8.5  # cm (distance between wheel centers)
MAX_SPEED = 1500

# Calibration parameters
CALIBRATION_WARMUP_MS = 15000  # 15 seconds
CALIBRATION_SAMPLES = 2000
OUTLIER_TRIM_PERCENT = 0.05  # Remove top/bottom 5%

# ZUPT parameters
ZUPT_ENABLED = True
ZUPT_MOTOR_THRESHOLD = 15  # Motors below this speed = stationary
ZUPT_GYRO_THRESHOLD = 0.15  # Gyro readings above this when "stationary" = bias drift
ZUPT_LEARNING_RATE = 0.05  # How quickly to adapt bias (0.05 = 5% per update)

# Gyro integration
UPDATE_PERIOD_MS = 20  # 50 Hz update rate

# ============================================================================
# GLOBAL STATE
# ============================================================================

# Gyro bias tracking
gyro_bias_z = 0.0  # Current best estimate of Z-axis bias (dps)
bias_lock = _thread.allocate_lock()

# Heading state
heading_angle = 0.0  # Current heading in degrees (0-360)
angle_lock = _thread.allocate_lock()

# System state
gyro_ready = False
last_motor_speeds = [0, 0]  # Track for ZUPT
motor_speed_lock = _thread.allocate_lock()

# Statistics (for debugging)
zupt_correction_count = 0
total_bias_correction = 0.0

# ============================================================================
# CALIBRATION
# ============================================================================

def calibrate_gyro(show_progress=True):
    """
    Perform high-quality gyro bias calibration with warmup and outlier rejection.

    Returns:
        float: Estimated Z-axis gyro bias in degrees per second
    """
    global gyro_bias_z

    if show_progress:
        display.fill(0)
        display.text("Gyro Warmup", 0, 10)
        display.text("KEEP STILL!", 0, 20)
        display.text("15 seconds...", 0, 30)
        display.show()

    print("Warming up gyro for 15 seconds...")
    time.sleep_ms(CALIBRATION_WARMUP_MS)

    # Collect samples
    samples = []
    if show_progress:
        display.fill(0)
        display.text("Calibrating...", 0, 10)
        display.show()

    print(f"Collecting {CALIBRATION_SAMPLES} samples...")
    for i in range(CALIBRATION_SAMPLES):
        imu.read()
        samples.append(imu.gyro.last_reading_dps[2])
        time.sleep_ms(1)

        # Progress indicator every 10%
        if show_progress and i % (CALIBRATION_SAMPLES // 10) == 0:
            display.fill(0)
            display.text("Calibrating...", 0, 10)
            display.text(f"{int(100*i/CALIBRATION_SAMPLES)}%", 0, 20)
            display.show()

    # Outlier rejection: remove top/bottom 5%
    samples.sort()
    trim_count = int(CALIBRATION_SAMPLES * OUTLIER_TRIM_PERCENT)
    trimmed = samples[trim_count:-trim_count] if trim_count > 0 else samples

    # Calculate bias
    bias = sum(trimmed) / len(trimmed)

    # Calculate standard deviation for quality check
    mean = bias
    variance = sum((x - mean) ** 2 for x in trimmed) / len(trimmed)
    std_dev = math.sqrt(variance)

    # Update global bias
    with bias_lock:
        gyro_bias_z = bias

    # Display results
    print(f"Calibration complete:")
    print(f"  Bias: {bias:.4f} dps")
    print(f"  Std Dev: {std_dev:.4f} dps")
    print(f"  Range: [{trimmed[0]:.4f}, {trimmed[-1]:.4f}]")

    if show_progress:
        display.fill(0)
        display.text("Bias:", 0, 10)
        display.text(f"{bias:.4f} dps", 0, 20)
        display.text(f"Dev: {std_dev:.4f}", 0, 30)
        display.text("Ready!", 0, 50)
        display.show()
        time.sleep_ms(2000)

    return bias


def warm_calibration():
    """
    Quick recalibration after motors have warmed up the system.
    Call this after 30-60 seconds of operation.
    """
    print("Performing warm recalibration...")
    old_bias = gyro_bias_z
    new_bias = calibrate_gyro(show_progress=False)

    drift = new_bias - old_bias
    print(f"Bias drift from warmup: {drift:.4f} dps")

    return new_bias


# ============================================================================
# GYRO INTEGRATION THREAD with ZUPT
# ============================================================================

def gyro_integration_thread():
    """
    Continuously integrates gyro readings to maintain heading estimate.
    Implements ZUPT to correct bias drift during stationary periods.
    """
    global heading_angle, gyro_bias_z, zupt_correction_count, total_bias_correction

    last_time = time.ticks_ms()

    # Wait for initial calibration
    while not gyro_ready:
        time.sleep_ms(10)

    print("Gyro integration thread started")

    while True:
        # Read IMU
        imu.read()
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, last_time) / 1000.0
        last_time = current_time

        # Get raw gyro reading
        raw_gyro_z = imu.gyro.last_reading_dps[2]

        # Get current bias estimate (thread-safe)
        with bias_lock:
            current_bias = gyro_bias_z

        # Correct for bias
        corrected_rate = raw_gyro_z - current_bias

        # === ZUPT: Zero-Velocity Update ===
        if ZUPT_ENABLED:
            # Check if robot should be stationary
            with motor_speed_lock:
                left_speed, right_speed = last_motor_speeds

            motors_stopped = (abs(left_speed) < ZUPT_MOTOR_THRESHOLD and
                            abs(right_speed) < ZUPT_MOTOR_THRESHOLD)

            if motors_stopped:
                # Robot should be stationary - any gyro reading is residual bias
                if abs(raw_gyro_z) > ZUPT_GYRO_THRESHOLD:
                    # Detected drift - update bias estimate
                    bias_correction = raw_gyro_z * ZUPT_LEARNING_RATE

                    with bias_lock:
                        gyro_bias_z += bias_correction

                    # Statistics
                    zupt_correction_count += 1
                    total_bias_correction += bias_correction

                    # Recalculate corrected rate with new bias
                    corrected_rate = raw_gyro_z - gyro_bias_z

        # Integrate to get heading
        with angle_lock:
            heading_angle += corrected_rate * dt
            # Wrap to 0-360 degrees
            heading_angle = heading_angle % 360

        time.sleep_ms(UPDATE_PERIOD_MS)


# ============================================================================
# HEADING ACCESS FUNCTIONS
# ============================================================================

def get_heading():
    """Get current heading estimate (thread-safe)."""
    with angle_lock:
        return heading_angle


def reset_heading(new_angle=0.0):
    """Reset heading to a specific value (thread-safe)."""
    global heading_angle
    with angle_lock:
        heading_angle = new_angle


def get_bias():
    """Get current gyro bias estimate (thread-safe)."""
    with bias_lock:
        return gyro_bias_z


def get_zupt_stats():
    """Get ZUPT correction statistics."""
    return {
        'correction_count': zupt_correction_count,
        'total_correction': total_bias_correction,
        'avg_correction': total_bias_correction / max(1, zupt_correction_count)
    }


# ============================================================================
# MOTOR CONTROL with ZUPT tracking
# ============================================================================

def set_motor_speeds(left_speed, right_speed):
    """
    Set motor speeds and update state for ZUPT.
    Use this instead of motors.set_speeds() directly.
    """
    global last_motor_speeds

    motors.set_speeds(int(left_speed), int(right_speed))

    with motor_speed_lock:
        last_motor_speeds = [left_speed, right_speed]


def stop_motors():
    """Stop motors and clear state."""
    set_motor_speeds(0, 0)


# ============================================================================
# MOTION CONTROL
# ============================================================================

def drive_straight(distance_cm, base_speed=600, Kp_enc=2.5, Kp_gyro=0.7):
    """
    Drive straight for a specified distance with gyro and encoder correction.

    Args:
        distance_cm: Distance to travel in cm (positive = forward, negative = backward)
        base_speed: Base motor speed
        Kp_enc: Proportional gain for encoder mismatch correction
        Kp_gyro: Proportional gain for heading correction
    """
    # Reset encoders
    encoders.get_counts(reset=True)
    distance_left = 0.0
    distance_right = 0.0

    # Record target heading
    target_heading = get_heading()

    # Conversion factors
    ticks_per_cm_left = (TICKS_PER_ROTATION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE_LEFT
    ticks_per_cm_right = (TICKS_PER_ROTATION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE_RIGHT

    # Direction
    forward = distance_cm >= 0
    target_distance = abs(distance_cm)

    print(f"Driving {'forward' if forward else 'backward'} {target_distance:.1f} cm")
    print(f"Target heading: {target_heading:.2f}°")

    while True:
        # Read encoder deltas
        delta_left, delta_right = encoders.get_counts(reset=True)

        # Convert to cm
        distance_left += delta_left / ticks_per_cm_left
        distance_right += delta_right / ticks_per_cm_right

        # Average distance
        avg_distance = (abs(distance_left) + abs(distance_right)) / 2

        # Encoder correction (fix wheel mismatch)
        error_enc = distance_left - distance_right
        correction_enc = Kp_enc * error_enc

        # Gyro correction (maintain heading)
        current_heading = get_heading()
        error_heading = target_heading - current_heading

        # Normalize heading error to [-180, 180]
        if error_heading > 180:
            error_heading -= 360
        elif error_heading < -180:
            error_heading += 360

        correction_gyro = Kp_gyro * error_heading

        # Calculate motor speeds
        speed_sign = 1 if forward else -1
        left_speed = speed_sign * base_speed - correction_enc - correction_gyro
        right_speed = speed_sign * base_speed + correction_enc + correction_gyro

        # Clamp speeds
        left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
        right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)

        # Apply motor speeds
        set_motor_speeds(left_speed, right_speed)

        # Display debug info
        display.fill(0)
        display.text(f"Dist: {avg_distance:.1f}/{target_distance:.1f}", 0, 0)
        display.text(f"Head: {current_heading:.1f}", 0, 10)
        display.text(f"Err: {error_heading:.2f}", 0, 20)
        display.text(f"L: {int(left_speed)}", 0, 30)
        display.text(f"R: {int(right_speed)}", 0, 40)
        display.show()

        # Check if target reached
        if avg_distance >= target_distance:
            break

        time.sleep_ms(10)

    # Stop
    stop_motors()
    print(f"Drive complete. Final heading: {get_heading():.2f}°")
    time.sleep_ms(100)  # Let ZUPT engage


def turn_to_angle(target_angle, base_speed=800, Kp=12, tolerance=2.0, validate_with_encoders=True):
    """
    Turn to an absolute heading angle with optional encoder validation.

    Args:
        target_angle: Target heading in degrees (0-360)
        base_speed: Base turning speed
        Kp: Proportional gain
        tolerance: Acceptable angle error in degrees
        validate_with_encoders: If True, compare gyro to encoder-derived angle
    """
    # Record starting position and heading
    encoders.get_counts(reset=True)
    start_heading = get_heading()

    print(f"Turning from {start_heading:.2f}° to {target_angle:.2f}°")

    # Track encoder-based angle if validating
    encoder_angle = 0.0

    while True:
        current_heading = get_heading()

        # Calculate error
        error = target_angle - current_heading

        # Normalize to [-180, 180]
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # If we're close enough, stop
        if abs(error) < tolerance:
            break

        # PID control (just P for now)
        turn_speed = Kp * error

        # Clamp speed
        turn_speed = max(min(turn_speed, base_speed), -base_speed)

        # Apply differential turning (left/right opposite)
        set_motor_speeds(-turn_speed, turn_speed)

        # Display
        display.fill(0)
        display.text(f"Targ: {target_angle:.1f}", 0, 0)
        display.text(f"Curr: {current_heading:.1f}", 0, 10)
        display.text(f"Err: {error:.2f}", 0, 20)
        display.text(f"Spd: {int(turn_speed)}", 0, 30)
        display.show()

        time.sleep_ms(10)

    # Stop
    stop_motors()

    # === ENCODER VALIDATION ===
    if validate_with_encoders:
        # Get final encoder counts
        delta_left, delta_right = encoders.get_counts(reset=True)

        # Calculate angle turned based on encoders
        # Turn angle = (right_distance - left_distance) / wheel_base
        dist_left = delta_left / ((TICKS_PER_ROTATION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE_LEFT)
        dist_right = delta_right / ((TICKS_PER_ROTATION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE_RIGHT)

        encoder_turn = (dist_right - dist_left) / WHEEL_BASE * (180.0 / math.pi)

        # Gyro-measured turn
        gyro_turn = get_heading() - start_heading
        if gyro_turn > 180:
            gyro_turn -= 360
        elif gyro_turn < -180:
            gyro_turn += 360

        # Compare
        validation_error = gyro_turn - encoder_turn

        print(f"Turn validation:")
        print(f"  Gyro turn: {gyro_turn:.2f}°")
        print(f"  Encoder turn: {encoder_turn:.2f}°")
        print(f"  Error: {validation_error:.2f}°")

        # If error is significant, adjust bias
        if abs(validation_error) > 3.0:  # More than 3° error
            # Estimate bias error (assuming turn took ~2 seconds)
            turn_duration = 2.0  # seconds (rough estimate)
            bias_error = validation_error / turn_duration

            print(f"Large turn error detected. Adjusting bias by {bias_error:.4f} dps")

            with bias_lock:
                gyro_bias_z += bias_error * 0.3  # Gentle 30% correction

    print(f"Turn complete. Final heading: {get_heading():.2f}°")
    time.sleep_ms(100)  # Let ZUPT engage


def turn_relative(degrees, base_speed=800, Kp=12, tolerance=2.0, validate_with_encoders=True):
    """
    Turn relative to current heading.

    Args:
        degrees: Degrees to turn (positive = right/CW, negative = left/CCW)
        base_speed: Base turning speed
        Kp: Proportional gain
        tolerance: Acceptable angle error
        validate_with_encoders: If True, use encoder cross-validation
    """
    target = (get_heading() + degrees) % 360
    turn_to_angle(target, base_speed, Kp, tolerance, validate_with_encoders)


# ============================================================================
# INITIALIZATION
# ============================================================================

def initialize(perform_warm_cal=False):
    """
    Initialize the robot system.

    Args:
        perform_warm_cal: If True, does a second calibration after warmup period
    """
    global gyro_ready

    print("=" * 50)
    print("StableGyro - Drift-Resistant Robot Control")
    print("=" * 50)

    # Initial calibration
    print("\n1. Performing initial gyro calibration...")
    calibrate_gyro(show_progress=True)

    # Start gyro integration thread
    print("\n2. Starting gyro integration thread...")
    gyro_ready = True
    _thread.start_new_thread(gyro_integration_thread, ())
    time.sleep_ms(500)

    print("\n3. System ready!")
    print(f"   Initial bias: {get_bias():.4f} dps")
    print(f"   ZUPT enabled: {ZUPT_ENABLED}")
    print(f"   Initial heading: {get_heading():.2f}°")

    display.fill(0)
    display.text("READY", 0, 20)
    display.text(f"Bias:{get_bias():.3f}", 0, 35)
    display.show()
    time.sleep_ms(1000)

    return True


# ============================================================================
# DIAGNOSTIC FUNCTIONS
# ============================================================================

def print_diagnostics():
    """Print system diagnostics."""
    stats = get_zupt_stats()

    print("\n" + "=" * 50)
    print("SYSTEM DIAGNOSTICS")
    print("=" * 50)
    print(f"Current heading: {get_heading():.2f}°")
    print(f"Current bias: {get_bias():.4f} dps")
    print(f"ZUPT corrections: {stats['correction_count']}")
    print(f"Total bias correction: {stats['total_correction']:.4f} dps")
    if stats['correction_count'] > 0:
        print(f"Average correction: {stats['avg_correction']:.4f} dps")
    print("=" * 50 + "\n")


def display_heading_loop(duration_seconds=10):
    """
    Display current heading on screen for testing.
    """
    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < duration_seconds * 1000:
        display.fill(0)
        display.text("Heading:", 0, 10)
        display.text(f"{get_heading():.2f} deg", 0, 25)
        display.text(f"Bias:{get_bias():.3f}", 0, 40)
        display.show()
        time.sleep_ms(50)
