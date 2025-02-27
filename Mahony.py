
from pololu_3pi_2040_robot import robot


display = robot.Display()
imu = robot.IMU()
imu.reset()
imu.enable_default()
import struct
import math
import time

twoKp = 2.0 * 0.5 # next one is kP
twoKi = 2.0 * 0.0 # next one is kP
q0 = 1.0
q1 = 0.0
q2 = 0.0
q3 = 0.0
integralFBx = 0.0
integralFBy = 0.0
integralFBz = 0.0
anglesComputed = 0.0
invSampleFreq = 1.0 / 1000

gxoff = gyoff = gzoff = 0.0
accxoff = accyoff = acczoff = 0.0

gx = gy = gz = ax = ay = az = mx = my = mz = 0


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
	q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3 = 0
	hx, hy, bx, bz = 0
	halfvx, halfvy, halfvz, halfwx, halfwy, halfwz = 0
	halfex, halfey, halfez = 0
	qa, qb, qc = 0

	# Use IMU algorithm if magnetometer measurement invalid
	# (avoids NaN in magnetometer normalisation)
	if(mx == 0) and (my == 0.0) and (mz == 0.0):
		# updateIMU(gx, gy, gz, ax,ay, az);
		return

	# Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533
	gy *= 0.0174533
	gz *= 0.0174533

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

def updateIMU(gx,  gy,  gz,  ax,  ay,  az):
	global q0, q1, q2, q3, integralFBx, integralFBy, integralFBz, twoKi, twoKp, anglesComputed
	recipNorm = 0
	halfvx, halfvy, halfvz = 0
	halfex, halfey, halfez = 0
	qa, qb, qc = 0

	# Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533
	gy *= 0.0174533
	gz *= 0.0174533

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
	global anglesComputed
	roll = math.atan2(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2)
	pitch = math.asin(-2.0 * (q1*q3 - q0*q2))
	yaw = math.atan2(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3)
	anglesComputed = 1

def gyroCalibration(numSamples=1000):
    global gxoff, gyoff, gzoff
    gx_total = gy_total = gz_total = 0.0
    gx = gy = gz = 0.0
    for i in range(numSamples):
        gx, gy, gz = imu.gyro.last_reading_dps
        gx_total += gx
        gy_total += gy
        gz_total += gz
        time.sleep_ms(1)

    gxoff = gx_total/numSamples
    gyoff = gy_total/numSamples
    gzoff = gz_total/numSamples
    return (gx_total/numSamples, gy_total, numSamples, gz_total/numSamples)

def accelCalibration(numSamples=1000):
    global accxoff, accyoff, acczoff
    ax_total = ay_total = az_total = 0
    for i in range(numSamples):
        gyroReading = imu.gyro.last_reading_dps
        ax_total += gyroReading[0]
        ay_total += gyroReading[1]
        az_total += gyroReading[2]
        time.sleep_ms(1)

    accxoff = accxoff/numSamples
    accyoff = accyoff/numSamples
    acczoff = acczoff/numSamples		
    return (ax_total/numSamples, ay_total/numSamples, az_total/numSamples)


while True:
    start = time.ticks_ms()
    gyroCalibration()
    accelCalibration()


    imu.read()
    g = imu.gyro.last_reading_dps
    a = imu.acc.last_reading_g
    m = imu.mag.last_reading_gauss  # Magnetometer values in Gauss
    ax, ay, az = a
    gx, gy, gz = g
    mx, my, mz = m



    magX = m[0]
    magY = m[1]
    magZ = m[2]
    yaw_mag = math.degrees(math.atan2(magY, magX))  # Yaw from magnetometer (degrees)



    display.fill(0)
    display.text("gx:", 0, 0)
    display.text(str(gx), 0, 10)
    display.text("gy:", 0, 23)
    display.text(str(gy), 0, 33)
    display.text("gz:", 0, 43)
    display.text(str(gz), 0, 53)
    display.show()
    
    dt = (time.ticks_ms() - start) / 1000.0  # Convert to seconds
