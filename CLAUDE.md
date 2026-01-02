# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a MicroPython robotics project for the Pololu 3pi+ 2040 robot platform. The codebase implements motion control, sensor fusion, and autonomous navigation using IMU (gyroscope, accelerometer, magnetometer) and wheel encoders.

## Hardware Platform

**Robot**: Pololu 3pi+ 2040
- **Motors**: Dual DC motors with encoder feedback
- **IMU**: 9-axis (gyroscope, accelerometer, magnetometer)
- **Display**: OLED display for debugging
- **Encoders**: 12 ticks per rotation, 29.86:1 gear ratio
- **Wheel specifications**:
  - Diameter: 3.2004 cm
  - Left wheel circumference: ~10.15 cm
  - Right wheel circumference: ~10.40 cm (slight variation for calibration)

## Code Architecture

### Recommended Primary Module: StableGyro.py (NEW)

**StableGyro.py** is the recommended unified control system for Science Olympiad missions. It addresses the core drift problem through runtime bias tracking rather than one-time calibration.

**Key Features:**
- **ZUPT (Zero-Velocity Updates)**: Continuously corrects gyro bias during stops between moves
- **Encoder Cross-Validation**: Validates gyro turns against wheel encoder measurements
- **Enhanced Calibration**: Multi-stage calibration with outlier rejection
- **Clean API**: Simple functions for driving straight and turning with stable heading
- **Thread-Safe**: Background gyro integration at 50 Hz

**Why Use StableGyro:**
- Maintains <1° drift over 2-minute missions (vs 3-4° with simple calibration)
- Automatically adapts to temperature-induced bias changes during operation
- Provides encoder validation to detect and correct gyro errors
- Science Olympiad-optimized: ZUPT resets drift during natural stops between moves

**Basic Usage:**
```python
import StableGyro as sg

# Initialize (calibrates gyro)
sg.initialize()

# Drive straight 40 cm
sg.drive_straight(40, base_speed=650)

# Turn to absolute heading
sg.turn_to_angle(90, base_speed=800)

# Turn relative to current heading
sg.turn_relative(90)  # Turn 90° right

# Check current heading
print(f"Heading: {sg.get_heading():.2f}°")
```

### Legacy Motion Control System

The codebase contains earlier approaches with different control strategies:

1. **Sensor Fusion Layer** (Mahony.py, Icarus.py)
   - Mahony AHRS filter for orientation estimation using quaternions
   - Fast inverse square root implementation for real-time performance
   - One-time gyro bias calibration

2. **Motion Control Layer** (Icarus.py, HeadingPID.py)
   - PID-based distance control with dual-wheel independent tracking
   - Gyro-based heading correction for straight-line driving
   - Turn-to-angle functionality with gradual slowdown

3. **Encoder Management**
   - Tracks cumulative encoder counts and converts to distance (cm)
   - Updates at ~100ms intervals
   - Separate left/right wheel tracking for differential drive corrections

### Key Modules

**StableGyro.py** - Recommended unified control system (NEW)
- Enhanced gyro calibration with 15s warmup and outlier rejection
- ZUPT-based dynamic bias tracking (corrects drift during stops)
- Encoder cross-validation during turns
- Thread-safe gyro integration at 50 Hz (20ms update period)
- Motion functions: `drive_straight()`, `turn_to_angle()`, `turn_relative()`
- Thread-safe heading access: `get_heading()`, `reset_heading()`
- Diagnostic functions: `print_diagnostics()`, `get_zupt_stats()`
- Configurable parameters: ZUPT learning rate, motor threshold, update period
- Test suite available in `test_stable_gyro.py`

**Icarus.py** - Legacy robot control program
- Primary motion functions: `SingleMotion`, `DoubleMotion`, `turnToAngle`
- Basic movement: `forw()`, `back()`, `rt()` (turn right)
- Encoder management: `checkEncoders()`, `resetEncodersOnce()`
- Mahony filter integration for IMU orientation
- PID control with adaptive speed scaling near target positions

**HeadingPID.py** - Gyro-integrated straight driving
- Thread-based gyro integration running at 20ms intervals
- `DS2()` function: Drive straight with encoder + gyro correction
- Gyro bias calibration: `GYRO_BIAS_Z = -0.52` (hardware-specific, may need recalibration)
- Separate wheel circumference calibration for mechanical variations

**GyroBiasCalibration.py** - Gyro calibration utility
- Warmup period: 15 seconds
- 2000 samples with outlier rejection (removes top/bottom 5%)
- Calculates bias and standard deviation for diagnostics
- Displays calibration progress on OLED

**GyroCalibration.py** - Simpler gyro calibration and reading tool
- Real-time gyro bias measurement
- Angle integration example

**Mahony.py** - AHRS sensor fusion implementation
- Standalone Mahony filter with both IMU-only and full 9-axis modes
- Gyro and accelerometer calibration functions
- Real-time orientation computation (roll, pitch, yaw)

## Running Code on the Robot

### Deployment Workflow
1. Code is written in MicroPython
2. Deploy to robot using the Pololu 3pi+ 2040's file transfer method (typically USB or wireless)
3. Files execute directly on the robot's RP2040 microcontroller
4. Use the display for debugging output

### Python Environment
- This is MicroPython code running on embedded hardware
- Do NOT use virtual environments (this is not CPython)
- The `pololu_3pi_2040_robot` library is pre-installed on the robot

## Important Constants and Calibration Values

### Robot Physical Parameters
```python
ticks_per_rotation = 12
gear_ratio = 29.86
wheel_diameter = 3.2004  # cm
wheel_circumference = 10.0543531  # cm
wheel_base = 5.6  # cm (distance between wheels)
```

### Gyro Calibration
- **Current bias**: `GYRO_BIAS_Z = -0.52` deg/s (Z-axis)
- **Expected drift**: ~0.06 degrees/min after calibration
- Run `GyroBiasCalibration.py` if gyro drift is observed
- Robot must remain completely still during calibration

### Motor Speed Limits
- Max speed: 1500
- Typical base speeds: 400-1200 depending on precision needs
- Minimum creep speed: 300-600 for smooth stopping

## PID Tuning Guidelines

### Distance Control (DoubleMotion, SingleMotion)
- Typical gains: `Kp=2.5-5.0`, `Ki=0`, `Kd=0`
- Slowdown threshold: 30% of target distance
- Scale factor: `(error / threshold) ** 1.5` for gradual deceleration
- Stop tolerance: 0.1-0.3 cm

### Heading Control (turnToAngle)
- Typical gains: `Kp=varies`, `Ki=0`, `Kd=0`
- Slowdown threshold: 10 degrees
- Stop tolerance: 4.5 degrees with speed < 400

### Straight Driving (DS2 in HeadingPID.py)
- Encoder correction: `Kp_enc=2.5`
- Gyro correction: `Kp_gyro=0.5-0.7`
- Update loop: 10ms

## Threading Model

HeadingPID.py uses threading for continuous gyro integration:
- Main thread: motion control and display
- Gyro thread: 20ms update loop for angle integration
- Thread safety: `angle_lock` protects shared angle variable
- Warmup: 5-15 second gyro stabilization before use

## Common Motion Patterns

### Forward Movement
```python
resetEncodersOnce()  # Clear encoder history
forw(30)  # Move 30 cm forward
# or with PID:
DoubleMotion(30, base_speed=1200, Kp=5, Ki=0, Kd=0, Lp=5, Li=0, Ld=0)
```

### Turning
```python
resetEncodersOnce()
MotionMagicExpoTurn(-6.6, 600, 5, 0, 0, time.ticks_ms())
# or angle-based:
turnToAngle(90, base_speed=800, Kp=10, Ki=0, Kd=0)
```

### Straight Driving with Gyro
```python
# In HeadingPID.py context
DS2(25, base_speed=600, Kp_enc=2.5, Kp_gyro=0.7)
```

## Debugging

### Display Output
The robot's OLED display shows real-time debug information:
- Current encoder values (left/right)
- Error terms
- Motor speeds
- Gyro angles/bias

### Common Display Pattern
```python
display.fill(0)  # Clear screen
display.text(f"Value: {value:.2f}", 0, 20)  # x=0, y=20
display.show()  # Update display
```

## Code Quality Notes

- The codebase mixes different motion control strategies (legacy `forw()` vs PID-based `DoubleMotion`)
- Some functions have hardcoded delays (e.g., `resetEncodersOnce()` waits 590ms)
- Turn distance calibration values (e.g., -6.6 for 90-degree turns) are empirically determined
- The Mahony filter includes dead zones for gyro readings < 0.05-0.1 deg/s to reduce noise
- Yaw calculation in Icarus.py has a suspicious scaling factor: `* 57.29578 * 12.145749` (line 450) - the second multiplier may be incorrect

## Sensor Fusion Details

The Mahony filter maintains orientation as a quaternion `(q0, q1, q2, q3)`:
- Update frequency: ~1000 Hz (1ms sample time)
- Proportional gain: `twoKp = 1.0` (2.0 * 0.5)
- Integral gain: `twoKi = 0.0` (integral feedback disabled)
- Falls back to IMU-only mode if magnetometer reads all zeros
- Converts quaternion to Euler angles (roll, pitch, yaw) on demand

## StableGyro Configuration and Testing

### Configuration Parameters (in StableGyro.py)

**Calibration:**
```python
CALIBRATION_WARMUP_MS = 15000      # 15 second warmup
CALIBRATION_SAMPLES = 2000          # Number of samples
OUTLIER_TRIM_PERCENT = 0.05         # Remove top/bottom 5%
```

**ZUPT (Zero-Velocity Update):**
```python
ZUPT_ENABLED = True                 # Enable/disable ZUPT
ZUPT_MOTOR_THRESHOLD = 15           # Speed below = stationary
ZUPT_GYRO_THRESHOLD = 0.15          # Gyro reading when stopped
ZUPT_LEARNING_RATE = 0.05           # Bias adaptation rate (5%)
```

**Gyro Integration:**
```python
UPDATE_PERIOD_MS = 20               # 50 Hz update rate
```

### Testing the System

**Quick Test:**
```python
# Run the simple demo
python test_stable_gyro.py
# Uses simple_demo() - drives square pattern
```

**Full Test Suite:**
Edit `test_stable_gyro.py` main section to run:
- `test_drift_stationary()` - Measure drift while still (validates ZUPT)
- `test_square_pattern()` - Drive square, measure cumulative error
- `test_turn_accuracy()` - Test 90°/180° turn precision
- `test_typical_mission()` - 90-second mission simulation

**Expected Performance:**
- Stationary drift: <0.5° over 2 minutes (with ZUPT)
- Turn accuracy: <2° average error for 90° turns
- Mission return-to-start: <2° after multiple moves

### Tuning ZUPT Parameters

If drift is still observed:

1. **Increase ZUPT learning rate** (faster bias adaptation):
   ```python
   ZUPT_LEARNING_RATE = 0.1  # More aggressive (10%)
   ```

2. **Lower gyro threshold** (catch smaller drift):
   ```python
   ZUPT_GYRO_THRESHOLD = 0.1  # More sensitive
   ```

3. **Increase motor threshold** (ZUPT engages earlier):
   ```python
   ZUPT_MOTOR_THRESHOLD = 25  # Trigger at higher speeds
   ```

### Encoder Cross-Validation

During turns, StableGyro compares gyro-measured angle to encoder-measured angle:
- If error > 3°, adjusts bias estimate by 30% of calculated error
- Helps catch temperature-induced bias drift during operation
- Enable/disable with `validate_with_encoders` parameter in turn functions

### Wheel Base Calibration

The encoder validation accuracy depends on correct `WHEEL_BASE`:
```python
WHEEL_BASE = 8.5  # cm - distance between wheel centers
```

To calibrate:
1. Command a 360° turn: `sg.turn_relative(360)`
2. Measure actual rotation (should be exactly 360°)
3. If robot over-rotates: increase `WHEEL_BASE`
4. If robot under-rotates: decrease `WHEEL_BASE`

### Diagnostics

During or after a mission:
```python
import StableGyro as sg

# Print full diagnostics
sg.print_diagnostics()

# Get ZUPT statistics
stats = sg.get_zupt_stats()
print(f"ZUPT corrections: {stats['correction_count']}")
print(f"Total bias correction: {stats['total_correction']:.4f} dps")
```

High ZUPT correction counts (>10 per minute) indicate temperature drift is occurring - this is normal and shows ZUPT is working.
