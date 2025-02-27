# Runs all the IMU sensors, displaying values on the screen and
# printing them to the USB serial port.

from pololu_3pi_2040_robot import robot
import time
import math

display = robot.Display()
imu = robot.IMU()
imu.reset()
imu.enable_default()

rateRoll = 0
ratePitch = 0
rateYaw = 0
rateCalibrationRoll = 0
rateCalibrationYaw = 0

rateCalibrationNumber = 0

accX = accY = accZ = 0
angleRoll = 0 
anglePitch = 0

kalmanAngleRoll = 0
kalmanUncertaintyAngleRoll = 2*2
kalmanUncertaintyAnglePitch = 2*2

kalmanOutput = [0,0]
lastGyroTime = time.ticks_ms()


gyroX_Off, gyroY_Off, gyroZ_Off = 0,0,0
accelX_Off, accelY_Off, accelZ_Off = 0,0,0

gx = gy = gz = 0.0 

def kalman1d(state, uncertainty, input, measurement):
    global kalmanOutput
    state = state + 0.004 * input
    uncertainty = uncertainty + 0.004 * 0.004 * 4 * 4
    gain = uncertainty * 1 / (1 * uncertainty + 3 * 3)
    state = state + gain * (measurement - state)
    uncertainty = (1 - gain) * uncertainty 

    kalmanOutput[0] = state
    kalmanOutput[1] = uncertainty



def gyroCalibration(numSamples=500):
    gx_total = gy_total = gz_total = 0.0
    gx = gy = gz = 0.0
    for i in range(numSamples):
        gx, gy, gz = imu.gyro.last_reading_dps
        gx_total += gx
        gy_total += gy
        gz_total += gz
        time.sleep_ms(1)
    return (gx_total/numSamples, gy_total, numSamples, gz_total/numSamples)

def accelCalibration(numSamples=500):
    ax_total = ay_total = az_total = 0
    for i in range(numSamples):
        gyroReading = imu.gyro.last_reading_dps
        ax_total += gyroReading[0]
        ay_total += gyroReading[1]
        az_total += gyroReading[2]
        time.sleep_ms(1)
    return (ax_total/numSamples, ay_total/numSamples, az_total/numSamples)

def getConvertedValues(accelTable):
    table = [0,0]
    table[0] = math.degrees(math.atan2(accelTable[1], math.sqrt(accelTable[0]**2 + accelTable[2]**2))) # roll
    table[1] = math.degrees(math.atan2(accelTable[0], math.sqrt(accelTable[1]**2 + accelTable[2]**2))) # pitch 
    
    return table

dt = 1 / 1000

while True:
    start = time.ticks_ms()
    gyroCalibration()
    accelCalibration()


    imu.read()
    g = imu.gyro.last_reading_dps
    a = imu.acc.last_reading_g
    m = imu.mag.last_reading_gauss  # Magnetometer values in Gauss

    accelMeasures = getConvertedValues(a)

    # roll_acc = math.degrees(math.atan2(a[1], math.sqrt(a[0]**2 + a[2]**2)))
    roll_acc = accelMeasures[0]
    pitch_acc = accelMeasures[1]

    
    gyrX = g[0] - gyroX_Off
    gyrY = g[1] - gyroY_Off
    gyrZ = g[2] - gyroZ_Off

    
    accX = a[0] - accelX_Off
    accY = a[1] - accelY_Off
    accZ = a[2] - accelZ_Off

    magX = m[0]
    magY = m[1]
    magZ = m[2]
    yaw_mag = math.degrees(math.atan2(magY, magX))  # Yaw from magnetometer (degrees)


    gx += gyrX * dt
    gy += gyrY * dt
    gz += gyrZ * dt

    alpha = 0.96
    gx = gx * alpha + roll_acc * (1 - alpha)
    gy = gy * alpha + pitch_acc * (1 - alpha)
    gz = gz * 0.9 + yaw_mag * (0.05) + accZ * (0.05)  # Combine gyro and magnetometer yaw
    
    display.fill(0)
    display.text("gx:", 0, 0)
    display.text(str(gx), 0, 10)
    display.text("gy:", 0, 23)
    display.text(str(gy), 0, 33)
    display.text("gz:", 0, 43)
    display.text(str(gz), 0, 53)
    display.show()
    
    dt = (time.ticks_ms() - start) / 1000.0  # Convert to seconds
