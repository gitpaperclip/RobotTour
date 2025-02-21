# This example makes the 3pi+ 2040 drive forward until it hits a wall, detect
# the collision with its bumpers, then reverse, turn, and keep driving.

from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time
import array

motors = robot.Motors()
encoders = robot.Encoders()

buzzer = robot.Buzzer()
display = robot.Display()
yellow_led = robot.YellowLED()
buttonA = robot.ButtonA()
buttonB = robot.ButtonB()
buttonC = robot.ButtonC()


max_speed = 1500
turn_time = 250
ticks_per_rotation = 12
gear_ratio = 29.86 
wheel_diameter = 3.2 #measure with calipers at mars. current units: Cm
wheel_circumference = 10.0531 #wheel circumference, current units: Cm

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

encoder_update_time = 10
display_update_time = 50

display.fill(0)
display.show()






time.sleep_ms(1000)
start_time = time.ticks_ms()

buzzer.play("a32 b32") #make this play a start sound effect to indicate start of the program.
# add display code.

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
    global distance_cm_left, distance_cm_right
    # reset the distance cm values in a 10 mili window
    currentTime = time.ticks_ms()
    firstCheckTime = time.ticks_ms()

    while (not finished):
        if (currentTime - firstCheckTime < 100):
            currentTime = time.ticks_ms()
            distance_cm_right = 0
            distance_cm_left = 0
        else: 
            finished = True
    



def checkEncodersWithWheelCommand(distanceRequired):
    global lastCheck, currentTime, leftCounts, rightCounts, prevLeftCounts, prevRightCounts, encoderAccess, distance_cm_right, distance_cm_left
    currentTime = time.ticks_ms()
    finished = False

    while (not finished): 
        arr = checkEncoders()  
        dl = arr[0]
        dr = arr[1]   

        if (dl < distanceRequired):
            motors.set_speeds(max_speed, max_speed)
        else:
            motors.set_speeds(0,0)
            finished = True

    prevLeftCounts = leftCounts
    prevRightCounts = rightCounts
    lastCheck = currentTime




while (not programFinished):

    display.fill(0)
    display.text(f"leftCounts {distance_cm_left}", 0, 20)
    display.text(f"rightCounts {distance_cm_right}", 0, 30)
    display.show()

    checkEncodersWithWheelCommand(30)
    resetEncodersOnce()
    time.sleep_ms(100)
    checkEncodersWithWheelCommand(60)
    resetEncodersOnce()
    time.sleep_ms(100)

    programFinished = True




    if buttonA.is_pressed():
        motors.set_speeds(max_speed, max_speed)
    else:
        motors.set_speeds(0, 0)

