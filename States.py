# This example makes the 3pi+ 2040 drive forward until it hits a wall, detect
# the collision with its bumpers, then reverse, turn, and keep driving.

from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time
import array
import math
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
    




def SingleMotion(target_distance, base_speed, Kp, Ki, Kd):
    global lastCheck, currentTime, leftCounts, rightCounts, prevLeftCounts, prevRightCounts
    global distance_cm_left, distance_cm_right

    currentTime = time.ticks_ms()
    prev_time = currentTime
    finished = False

    integral = 0
    prev_error = 0
    min_speed = 300  # Lower creep speed to blend stop better
    slow_down_threshold = target_distance * 0.3  # Start slowing down at 30% of the total distance

    while (not finished):
        # Read encoder values
        arr = checkEncoders()  
        dl = arr[0]  # Left wheel distance
        dr = arr[1]
        davg = (dl + dr) / 2

        # Compute error (how far we still need to move)
        error = target_distance - davg
        dt = (time.ticks_ms() - prev_time) / 1000.0  # Convert to seconds

        if dt <= 0:
            dt = 0.001  # Prevent division by zero

        integral += error * dt
        derivative = (error - prev_error) / dt

        # **Earlier slowdown** (Start decelerating smoothly when within 30% of target)
        if abs(error) < slow_down_threshold:
            scale_factor = (abs(error) / slow_down_threshold) ** 1.5  # More gradual than before
        else:
            scale_factor = 1  # Full speed until slowdown range

        # PID raw output
        speed = (Kp * error) + (Ki * integral) + (Kd * derivative)

        # Apply gradual speed reduction
        adjusted_speed = speed * scale_factor

        # **Adaptive Minimum Speed** (Ensures smooth slow crawling near target)
        creep_speed = max(min_speed, base_speed * (abs(error) / slow_down_threshold))
        if abs(adjusted_speed) < creep_speed and abs(error) > 0.2:
            adjusted_speed = creep_speed * (1 if adjusted_speed > 0 else -1)

        # Ensure the speed is within motor limits
        adjusted_speed = max(min(adjusted_speed, base_speed), -base_speed)

        # Display debug info
      #  display.fill(0)
      #  display.text(f"Pos {dl:.2f}", 0, 20)
      #  display.text(f"Err {error:.2f}", 0, 30)
      #  display.text(f"Speed {adjusted_speed:.2f}", 0, 40)
      #  display.show()
        display.fill(0)
        display.text(f"dr {dr:.2f}", 0, 20)
        display.text(f"dl {dr:.2f}", 0, 30)
        display.text(f"targ {target_distance:.2f}", 0, 40)
        display.text(f"Err {error:.2f}", 0, 50)
        display.text(f"Speed {adjusted_speed:.2f}", 0, 60)
        display.show()
        # Apply speed
        motors.set_speeds(adjusted_speed, adjusted_speed + (adjusted_speed * 0.01))

        # **Blend into stopping instead of jerking**
        if abs(error) < 0.1:  # Smooth final stop
            motors.set_speeds(20, 20)  # Light final push
            time.sleep_ms(50)  # Let it settle
            motors.set_speeds(0, 0)
            finished = True

        prev_error = error
        prev_time = time.ticks_ms()
        time.sleep_ms(10)  # Control loop delay


def DoubleMotion(target_distance, base_speed, Kp, Ki, Kd, Lp, Li, Ld):
    global lastCheck, currentTime, leftCounts, rightCounts, prevLeftCounts, prevRightCounts
    global distance_cm_left, distance_cm_right

    currentTime = time.ticks_ms()
    prev_time = currentTime
    finished = False
    Lfinished = True

    integral = 0
    prev_error = 0
    min_speed = 600  # Lower creep speed to blend stop better
    slow_down_threshold = target_distance * 0.3  # Start slowing down at 30% of the total distance
    
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




def MotionMagicExpoTurn(target_distance, base_speed, Kp, Ki, Kd):
    global lastCheck, currentTime, leftCounts, rightCounts, prevLeftCounts, prevRightCounts

    currentTime = time.ticks_ms()
    prev_time = currentTime
    finished = False

    integral = 0
    prev_error = 0
    min_speed = 600  # Lower creep speed to blend stop better
    slow_down_threshold = target_distance * 0.01  # Start slowing down at 30% of the total distance


    dt = 1 / 1
    while (not finished):
        # Read encoder values
        arr = checkEncoders()  
        dl = arr[0]  # Left wheel ticks
        dr = arr[1]
        davg = abs((dl + dr) / 2)

        # Compute error (how far we still need to move)
        error = target_distance - dl
        dt = (time.ticks_ms() - prev_time) / 1000.0  # Convert to seconds

        if dt <= 0:
            dt = 0.001  # Prevent division by zero

        integral += error * dt
        derivative = (error - prev_error) / dt

        # **Earlier slowdown** (Start decelerating smoothly when within 30% of target)
        if abs(error) < slow_down_threshold:
            scale_factor = (abs(error) / slow_down_threshold) ** 1.5  # More gradual than before
        else:
            scale_factor = 1  # Full speed until slowdown range

        # PID raw output
        speed = (Kp * error) + (Ki * integral) + (Kd * derivative)

        # Apply gradual speed reduction
        adjusted_speed = speed * scale_factor

        # **Adaptive Minimum Speed** (Ensures smooth slow crawling near target)
        creep_speed = max(min_speed, base_speed * (abs(error) / slow_down_threshold))
        if abs(adjusted_speed) < creep_speed and abs(error) > 0.2:
            adjusted_speed = creep_speed * (1 if adjusted_speed > 0 else -1)

        # Ensure the speed is within motor limits
        adjusted_speed = max(min(adjusted_speed, base_speed), -base_speed)

        # Display debug info
        display.fill(0)
        display.text(f"dr {dr:.2f}", 0, 20)
        display.text(f"dl {dr:.2f}", 0, 30)
        display.text(f"targ {target_distance:.2f}", 0, 40)
        display.text(f"Err {error:.2f}", 0, 50)
        display.text(f"Speed {adjusted_speed:.2f}", 0, 60)
        display.show()

        # Apply speed
        motors.set_speeds(adjusted_speed, -adjusted_speed + (adjusted_speed * 0.01))

        # **Blend into stopping instead of jerking**
        if abs(error) < 0.05:  # Smooth final stop
            motors.set_speeds(20, 20)  # Light final push
            time.sleep_ms(50)  # Let it settle
            motors.set_speeds(0, 0)
            finished = True

        prev_error = error
        prev_time = time.ticks_ms()



while (not programFinished):
    resetEncodersOnce()
   # MotionMagicQuadratic(-50, 1500, 80, 0, 0)
    DoubleMotion(50, 1000, 180, 0, 0, 180, 0, 0)
    resetEncodersOnce()
    MotionMagicExpoTurn(6.75, 500, 2, 0, 0)
    resetEncodersOnce()
    DoubleMotion(50, 1000, 180, 0, 0, 180, 0, 0)
    resetEncodersOnce()
    MotionMagicExpoTurn(6.75, 500, 2, 0, 0)
    resetEncodersOnce()
    DoubleMotion(50, 1000, 180, 0, 0, 180, 0, 0)
    resetEncodersOnce()
    MotionMagicExpoTurn(6.75, 500, 2, 0, 0)
    resetEncodersOnce()
    DoubleMotion(50, 1000, 180, 0, 0, 180, 0, 0)
    resetEncodersOnce()
    MotionMagicExpoTurn(6.75, 500, 2, 0, 0)

   # MotionMagicExpoTurn(6.75, 500, 2, 0, 0)
    #resetEncodersOnce()
   # time.sleep_ms(500)
   # MotionMagicExpoTurn(6.75, 500, 2, 0, 0)
  #  resetEncodersOnce()
   # time.sleep_ms(500)
   # MotionMagicExpoTurn(6.75, 500, 2, 0, 0)
   # resetEncodersOnce()
   # time.sleep_ms(500)
   # MotionMagicExpoTurn(6.75, 500, 2, 0, 0)
   # resetEncodersOnce()
   # time.sleep_ms(500)




    #resetEncodersOnce()
    #time.sleep_ms(100)

   # DoubleMotion(-50, 500, 180, 0, 0, 180, 0, 0)

   # resetEncodersOnce()
   # MotionMagicExpoTurn(6.32, 500, 1, 0, 0)
   # time.sleep_ms(500)
    programFinished = True
    #display.fill(0)
  #  display.text(f"Pos {arr[0]:.2f}", 0, 20)
   # display.text(f"Err {arr[1]:.2f}", 0, 30)
    display.show()
    #turn(100, 1400, True)
    #time.sleep_ms(100)
   # resetEncodersOnce()
   # checkEncodersWithWheelCommand(60, 1500)
   # resetEncodersOnce()
   # time.sleep_ms(100)





   # if buttonA.is_pressed():

   # else:
     #   motors.set_speeds(0, 0)

    #programFinished = True

