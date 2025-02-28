
from pololu_3pi_2040_robot import robot


display = robot.Display()
imu = robot.IMU()
motors = robot.Motors()
imu.reset()
imu.enable_default()
encoders = robot.Encoders()
import struct
import math
import _thread
import time

yawOffset = 0
correctedYaw = 0
twoKp = 2.0 * 20  # next one is kP
twoKi = 2.0 * 0.1 # next one is kP
q0 = 1.0
q1 = 0.0
q2 = 0.0
q3 = 0.0
integralFBx = 0.0
integralFBy = 0.0
integralFBz = 0.0


max_speed = 1500
turn_time = 250
ticks_per_rotation = 12
gear_ratio = 29.86 
wheel_diameter = 3.2004 #measure with calipers at mars. current units: Cm
wheel_circumference = 10.0543531 #wheel circumference, current units: Cm

# MOtor stuff
distance_cm_left = 0
distance_cm_right = 0


programFinished = False
currentTime = 0
lastCheck = 0

encoderAccess = [0,0]

leftCounts = 0
rightCounts = 0

prevLeftCounts = 0
prevRightCounts = 0


TLCounts = 0
TRCounts = 0
TLCountsPrev = 0
TRCountsPrev = 0
TencoderAccess = [0,0]

RencoderTicks = 0
LencoderTicks = 0


encoder_update_time = 1
display_update_time = 50

display.fill(0)
display.show()



gOffsets = [0.5, 0.4 , -0.5]
aOffsets = [-0.01, -0.01, 1.02]
magOffsets = [-1.59, -0.38, -1.21]
anglesComputed = 0.0
invSampleFreq = 1.0 / 1000

roll = 0
angle = 0
pitch = 0
yaw = 0
trueHeading = 0

gxoff = gyoff = gzoff = 0.0
accxoff = accyoff = acczoff = 0.0

gx = gy = gz = ax = ay = az = mx = my = mz = 0
turn_rate = 0




def checkEncoders():
    global lastCheck, currentTime, leftCounts, rightCounts, prevLeftCounts, prevRightCounts, encoderAccess, distance_cm_right, distance_cm_left
    distanceArray = [0,0]
    currentTime = time.ticks_ms()
    if (currentTime > (lastCheck + encoder_update_time)):
        leftEncoderAccess = encoders.get_counts(reset = True)
        rightCounts += leftEncoderAccess[1]
        leftCounts += leftEncoderAccess[0] 

        distance_cm_left += ((leftCounts - prevLeftCounts) / (ticks_per_rotation * gear_ratio) * wheel_circumference)
        distance_cm_right += ((rightCounts - prevRightCounts) / (ticks_per_rotation * gear_ratio) * wheel_circumference)
        
        prevLeftCounts = leftCounts
        prevRightCounts = rightCounts
        lastCheck = currentTime
        distanceArray = [distance_cm_left, distance_cm_right]
    return distanceArray



def resetEncodersOnce():
    finished = False
    global distance_cm_left, distance_cm_right, TRCounts, TLCounts
    # reset the distance cm values in a 10 mili window
    currentTime = time.ticks_ms()
    firstCheckTime = time.ticks_ms()

    while (not finished):
        if (currentTime - firstCheckTime < 100):
            currentTime = time.ticks_ms()
            distance_cm_right = 0
            distance_cm_left = 0
            TRCounts = 0
            TLCounts = 0
        else: 
            finished = True
    



def DoubleMotion(target_distance, base_speed, Kp, Ki, Kd, Lp, Li, Ld):
    global lastCheck, currentTime, leftCounts, rightCounts, prevLeftCounts, prevRightCounts
    global distance_cm_left, distance_cm_right

    currentTime = time.ticks_ms()
    prev_time = currentTime
    finished = False
    Lfinished = True

    integral = 0
    prev_error = 0
    min_speed = 400  # Lower creep speed to blend stop better
    slow_down_threshold = target_distance * 0.8  # Start slowing down at 30% of the total distance
    
    L_integral = 0
    L_prev_error = 0

    while (not (finished and Lfinished)):
        # Read encoder values
        arr = checkEncoders()  
        dl = arr[0]  # Left wheel distance
        dr = arr[1]
        davg = (dl + dr) / 2

        # Compute error (how far we still need to move)
        error = target_distance - dr
        L_error = target_distance - dl
        dt = (time.ticks_ms() - prev_time) / 1000.0  # Convert to seconds

        if dt <= 0:
            dt = 0.001  # Prevent division by zero

        integral += error * dt
        L_integral += L_error * dt

        derivative = (error - prev_error) / dt
        L_derivative = (L_error - L_prev_error) / dt

        # **Earlier slowdown** (Start decelerating smoothly when within 30% of target)
        if abs(error) < slow_down_threshold:
            scale_factor = (abs(error) / slow_down_threshold) ** 1.5  # More gradual than before
        else:
            scale_factor = 1  # Full speed until slowdown range

        if abs(L_error) < slow_down_threshold:
            L_scale_factor = (abs(L_error) / slow_down_threshold) ** 1.5  # More gradual than before
        else:
            L_scale_factor = 1  # Full speed until slowdown range

        # PID raw output
        speed = (Kp * error) + (Ki * integral) + (Kd * derivative)
        L_speed = (Lp * L_error) + (Li * L_integral) + (Ld * L_derivative)

        # Apply gradual speed reduction
        adjusted_speed = speed * scale_factor
        L_adjusted_speed = L_speed * L_scale_factor

        # **Adaptive Minimum Speed** (Ensures smooth slow crawling near target)
        creep_speed = max(min_speed, base_speed * (abs(error) / slow_down_threshold))
        if abs(adjusted_speed) < creep_speed and abs(error) > 0.2:
            adjusted_speed = creep_speed * (1 if adjusted_speed > 0 else -1)

        L_creep_speed = max(min_speed, base_speed * (abs(L_error) / slow_down_threshold))
        if abs(L_adjusted_speed) < creep_speed and abs(L_error) > 0.2:
            L_adjusted_speed = creep_speed * (1 if L_adjusted_speed > 0 else -1)

        # Ensure the speed is within motor limits
        adjusted_speed = max(min(adjusted_speed, base_speed), -base_speed)

        L_adjusted_speed = max(min(L_adjusted_speed, base_speed), -base_speed)

        # Display debug info
      #  display.fill(0)
      #  display.text(f"Pos {dl:.2f}", 0, 20)
      #  display.text(f"Err {error:.2f}", 0, 30)
      #  display.text(f"Speed {adjusted_speed:.2f}", 0, 40)
      #  display.show()
        display.fill(0)
       # display.text(f"dr {dr:.2f}", 0, 20)
       # display.text(f"dl {dr:.2f}", 0, 30)
        display.text(f"T {target_distance:.2f}", 0, 60)
        display.text(f"RE {error:.2f}", 0, 20)
        display.text(f"LE {L_error:.2f}", 0, 30)
        display.text(f"RS {adjusted_speed:.2f}", 0, 40)
        display.text(f"LS {L_adjusted_speed:.2f}", 0, 50)

        display.show()
        # Apply speed
        motors.set_right_speed(adjusted_speed)
        motors.set_left_speed(L_adjusted_speed)



        if abs(error) < 0.1 and abs(adjusted_speed) < min_speed:  # Only stop if speed is also small
            finished = True

        if abs(L_error) < 0.1 and abs(L_adjusted_speed) < min_speed:
            Lfinished = True

        prev_error = error
        L_prev_error = L_error
        prev_time = time.ticks_ms()
        time.sleep_ms(1)  # Control loop delay




def inv_sqrt(x: float) -> float:
    halfx = 0.5 * x
    i = struct.unpack('i', struct.pack('f', x))[0]  # Interpret float as int
    i = 0x5f3759df - (i >> 1)  # Magic number approximation
    y = struct.unpack('f', struct.pack('i', i))[0]  # Convert back to float
    y = y * (1.5 - (halfx * y * y))  # First Newton-Raphson iteration
    y = y * (1.5 - (halfx * y * y))  # Second iteration (optional)
    return y

def Mahony(gx, gy,  gz, ax, ay, az, mx, my, mz):
	global q0, q1, q2, q3, integralFBx, integralFBy, integralFBz, twoKi, twoKp, anglesComputed
	recipNorm = 0
	q0q0 = 0
	q0q1 = 0 
	q0q2 = 0 
	q0q3 = 0 
	q1q1 = 0 
	q1q2 = 0 
	q1q3 = 0 
	q2q2 = 0 
	q2q3 = 0  
	q3q3 = 0
	hx = 0 
	hy = 0 
	bx = 0 
	bz = 0
	halfvx = 0 
	halfvy = 0 
	halfvz = 0 
	halfwx = 0
	halfwy = 0 
	halfwz = 0
	halfex = 0
	halfey = 0
	halfez = 0
	qa = 0  
	qb = 0
	qc = 0

	# Use IMU algorithm if magnetometer measurement invalid
	# (avoids NaN in magnetometer normalisation)
	if(mx == 0) and (my == 0.0) and (mz == 0.0):
		updateIMU(gx, gy, gz, ax,ay, az)
		return

	# Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533
	gy *= 0.0174533
	gz *= 0.0174533

	if(gx < 0.05):
		gx = 0
	if(gy < 0.05):
		gy = 0
	if(gz < 0.05):
		gz = 0
		
	# Compute feedback only if accelerometer measurement valid
	# (avoids NaN in accelerometer normalisation)
	if(not ((ax == 0.0) and (ay == 0.0) and (az == 0.0))):
		recipNorm = inv_sqrt(ax * ax + ay * ay + az * az)
		ax *= recipNorm
		ay *= recipNorm
		az *= recipNorm

	# Normalise magnetometer measurement
		recipNorm = inv_sqrt(mx * mx + my * my + mz * mz)
		mx *= recipNorm
		my *= recipNorm
		mz *= recipNorm

    # Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0
		q0q1 = q0 * q1
		q0q2 = q0 * q2
		q0q3 = q0 * q3
		q1q1 = q1 * q1
		q1q2 = q1 * q2
		q1q3 = q1 * q3
		q2q2 = q2 * q2
		q2q3 = q2 * q3
		q3q3 = q3 * q3

	 # Reference direction of Earth's magnetic field
		hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2))
		hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1))
		bx = math.sqrt(hx * hx + hy * hy)
		bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2))

	 # Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2
		halfvy = q0q1 + q2q3
		halfvz = q0q0 - 0.5 + q3q3
		halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2)
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3)
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2)

		# Error is sum of cross product between estimated direction
		# and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy)
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz)
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx)
		
		# Compute and apply integral feedback if enabled
		if (twoKi > 0.0):
			e = 0
			integralFBx += twoKi * halfex * invSampleFreq
			integralFBy += twoKi * halfey * invSampleFreq
			integralFBz += twoKi * halfez * invSampleFreq
			gx += integralFBx  # apply integral feedback
			gy += integralFBy
			gz += integralFBz
		else:
			integralFBx = 0.0
			integralFBy = 0.0
			integralFBz = 0.0

		gx += twoKp * halfex
		gy += twoKp * halfey
		gz += twoKp * halfez
	

	# Integrate rate of change of quaternion
	gx *= (0.5 * invSampleFreq)		# pre-multiply common factors
	gy *= (0.5 * invSampleFreq)
	gz *= (0.5 * invSampleFreq)
	qa = q0
	qb = q1
	qc = q2
	q0 += (-qb * gx - qc * gy - q3 * gz)
	q1 += (qa * gx + qc * gz - q3 * gy)
	q2 += (qa * gy - qb * gz + q3 * gx)
	q3 += (qa * gz + qb * gy - qc * gx)

	# Normalise quaternion
	recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
	q0 *= recipNorm
	q1 *= recipNorm
	q2 *= recipNorm
	q3 *= recipNorm
	anglesComputed = 0

def isNear(val, targ):
    value = False
    if (val > targ):
        if val - targ < 0.1:
            value = True
    elif (val < targ):
        if targ - val < 0.05:
            value = True
    return value

def updateIMU(gx,  gy,  gz,  ax,  ay,  az):
	global q0, q1, q2, q3, integralFBx, integralFBy, integralFBz, twoKi, twoKp, anglesComputed
	recipNorm = 0
	halfvx = halfvy = halfvz = 0
	halfex = halfey = halfez = 0
	qa = qb = qc = 0

	# Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533
	gy *= 0.0174533
	gz *= 0.0174533
	if (abs(gx) < 0.1):
		gx = 0
	if (abs(gy) < 0.1):
		gy = 0
	if (abs(gz) < 0.1):
		gz = 0
				
	# Compute feedback only if accelerometer measurement valid
	# (avoids NaN in accelerometer normalisation)
	if(not ((ax == 0.0) and (ay == 0.0) and (az == 0.0))):
		# Normalise accelerometer measurement
		recipNorm = inv_sqrt(ax * ax + ay * ay + az * az)
		ax *= recipNorm
		ay *= recipNorm
		az *= recipNorm

		# Estimated direction of gravity
		halfvx = q1 * q3 - q0 * q2
		halfvy = q0 * q1 + q2 * q3
		halfvz = q0 * q0 - 0.5 + q3 * q3

		# Error is sum of cross product between estimated
		#and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy)
		halfey = (az * halfvx - ax * halfvz)
		halfez = (ax * halfvy - ay * halfvx)

		# Compute and apply integral feedback if enabled
		if(twoKi > 0.0):
			# integral error scaled by Ki
			integralFBx += twoKi * halfex * invSampleFreq
			integralFBy += twoKi * halfey * invSampleFreq
			integralFBz += twoKi * halfez * invSampleFreq
			gx += integralFBx
			gy += integralFBy
			gz += integralFBz
		else:
			integralFBx = 0.0
			integralFBy = 0.0
			integralFBz = 0.0

		# Apply proportional feedback
		gx += twoKp * halfex
		gy += twoKp * halfey
		gz += twoKp * halfez
	

	# Integrate rate of change of quaternion
	gx *= (0.5 * invSampleFreq)
	gy *= (0.5 * invSampleFreq)
	gz *= (0.5 * invSampleFreq)
	qa = q0
	qb = q1
	qc = q2
	q0 += (-qb * gx - qc * gy - q3 * gz)
	q1 += (qa * gx + qc * gz - q3 * gy)
	q2 += (qa * gy - qb * gz + q3 * gx)
	q3 += (qa * gz + qb * gy - qc * gx)

	# Normalise quaternion
	recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
	q0 *= recipNorm
	q1 *= recipNorm
	q2 *= recipNorm
	q3 *= recipNorm
	anglesComputed = 0


def computeAngles():
	global anglesComputed, roll, pitch, yaw
	roll = math.atan2(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2) * 57.29578
	pitch = math.asin(-2.0 * (q1*q3 - q0*q2)) * 57.29578 
	yaw = math.atan2(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3) * 57.29578  * 12.145749
	anglesComputed = 1

def turn_90_degrees_pid(target_angle, Kp):
    global yaw
    finished = False
    initial_yaw = yaw  # Current yaw
    target_yaw = initial_yaw + target_angle

    # Normalize target yaw to 0-360 degrees
    if target_yaw > 360:
        target_yaw -= 360
    elif target_yaw < 0:
        target_yaw += 360

    while not finished:
        # Continuously update IMU data in this loop
        current_yaw = yaw  # Get the current yaw
        error = target_yaw - current_yaw
        imu.read()
        g = imu.gyro.last_reading_dps
        a = imu.acc.last_reading_g
        m = imu.mag.last_reading_gauss  # Magnetometer values in Gauss
        ax, ay, az = a
        gx, gy, gz = g
        updateIMU(gx, gy, gz, ax, ay, az)
        computeAngles()
        display.fill(0)
        display.text(f"true yaw {target_angle}", 0, 20)
        display.text(f"Err {error}", 0, 30)
        display.show()
        # Normalize error to -180 to 180 range
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        
        # PID control logic (you'll want to implement this here)
        pid_output = Kp * error  # Simple proportional control (adjust as needed)
        motors.set_speeds(-pid_output, pid_output)
        
        # Break if we're within an acceptable error threshold
        if abs(error) < 2:
            motors.set_speeds(0, 0) # Stop the motors when the target is reached
            finished = True
        
        time.sleep(0.01)  # Sleep to avoid excessive CPU usage


def turnToAngle(target_angle, base_speed, Kp, Ki, Kd):
    """
    Turn the robot to the target angle using PID control.
    
    Parameters:
        target_angle (float): The target angle in degrees.
        base_speed (float): The base speed for turning.
        Kp (float): Proportional gain.
        Ki (float): Integral gain.
        Kd (float): Derivative gain.
    """
    global yaw, imu
    prev_time = time.ticks_ms()
    prev_error = 0
    integral = 0
    finished = False
    min_speed = 400  # Minimal speed for creep control
    slow_down_threshold = 10  # Threshold for slowing down when close to target angle


    while not finished:
        imu.read()
        g = imu.gyro.last_reading_dps
        a = imu.acc.last_reading_g
        m = imu.mag.last_reading_gauss  # Magnetometer values in Gauss
        ax, ay, az = a
        gx, gy, gz = g
        updateIMU(gx, gy, gz, ax, ay, az)
        computeAngles()        
        # Get current yaw angle from IMU
        current_angle = yaw  # Get this value from your IMU or another source
        
        # Compute the angle error
        error = target_angle - current_angle
        
        # Normalize error to the range of -180 to 180 degrees
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        
        # Calculate time difference
        current_time = time.ticks_ms()
        dt = (current_time - prev_time) / 1000.0  # Time in seconds
        if dt <= 0:
            dt = 0.001  # Avoid division by zero
        
        # Update the integral term and the derivative term
        integral += error * dt
        derivative = (error - prev_error) / dt
        
        # PID control output
        pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative)
        
        # Slow down as we approach the target angle
        if abs(error) < slow_down_threshold:
            scale_factor = (abs(error) / slow_down_threshold) ** 1.5  # More gradual slowdown
        else:
            scale_factor = 1  # Full speed until near the target
        
        # Apply scale factor
        adjusted_speed = pid_output * scale_factor
        
        # Ensure the adjusted speed stays within reasonable limits
        adjusted_speed = max(min(adjusted_speed, base_speed), -base_speed)
        
        # Apply speed to motors (adjust for turning direction)
        if adjusted_speed > 0:
            motors.set_left_speed(-adjusted_speed)
            motors.set_right_speed(adjusted_speed)  # Opposite direction for turning right
        else:
            motors.set_left_speed(-adjusted_speed)
            motors.set_right_speed(adjusted_speed)  # Opposite direction for turning left
        
        # Display some debug information if needed
        display.fill(0)
        display.text(f"Err: {error:.2f}", 0, 20)
        display.text(f"Speed: {adjusted_speed:.2f}", 0, 30)
        display.show()
        
        # Check if we've reached the target angle (within a small tolerance)
        if abs(error) < 1:
            finished = True
        
        # Update previous error and time
        prev_error = error
        prev_time = current_time
        
        # Small delay to avoid overloading the control loop
        time.sleep_ms(1)




while not programFinished:
    start = time.ticks_ms()
    turnToAngle(90, 600, 100, 0, 0)
    display.fill(0)
    #DoubleMotion(30, 800, 1100, 0, 0, 1100, 0, 0.02)
	
    #Mahony(gx, gy, gz, ax, ay, az, mx, my, mz)



    
