from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
from pybricks.parameters import Button, Port, Stop
from pybricks.tools import StopWatch, wait
#from pybricks.robotics import DriveBase

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the components.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
gyro = GyroSensor(Port.S1)
gyro.reset_angle(0)
color_sensor = ColorSensor(Port.S2)
arm_motor = Motor()
arm_motor.control.limits(None,150,None)

# Initialize the drive base.
#robot = DriveBase(left_motor, right_motor, wheel_diameter=68.8, axle_track=127)

# line follow variables
target_value = 50
KP = 1.0
KD = 0.0
KI = 0.0
prev_error = 0
integral = 0

SPEED = 50 # normal straight line speed

def init_arm():
    # lower arm to the ground
    lower_arm()
    arm_motor.reset_angle(0)

def calibrateTargetValue():
    global target_value

    min = 100
    max = 0
    sum_min = 0
    sum_max = 0
    left_motor.run(50)
    right_motor.run(-50)

    for i in range(3):
        for j in range(5000):
            reading = color_sensor.reflection()

            if reading < min:
                min = reading
            elif reading > max:
                max = reading
        sum_min += min
        sum_max += max
    
    avg_min = sum_min / 3
    avg_max = sum_max / 3
    left_motor.stop()
    right_motor.stop()

    target_value = (avg_min + avg_max) / 2

def follow_line(): # makes robot turn to follow the line
    global prev_error, integral

    sensor_value = color_sensor.reflection()

    error = target_value - sensor_value

    derivative = error - prev_error

    integral += error

    steering_value = error * KP + derivative * KD + integral * KI
    left_motor.run(SPEED+steering_value)
    right_motor.run(SPEED-steering_value)

    prev_error = error

    arm_motor.run_target(100,-100)

def raise_arm():
    arm_motor.run_until_stalled() # TO FINISH

def lower_arm():
    arm_motor.run_until_stalled(200, Stop.COAST, 50) # DUTY LIMIT TO CHANGE

def pickup_cube(): # TO IMPROVE
    left_motor.run_until_stalled(200, Stop.COAST, 20)
    right_motor.run_until_stalled(200, Stop.COAST, 20)

    wait(100)
    
    raise_arm()

    wait(100)

    #Drive back and lower arm
    


# test program
while True:

    while not any(ev3.buttons.pressed()):
        wait(10)
    
    if Button.CENTER in ev3.buttons.pressed():
        stopwatch = StopWatch()

        while stopwatch.time < 10000:
            follow_line()

        left_motor.stop()
        right_motor.stop()

    elif Button.UP in ev3.buttons.pressed():
        calibrate_target_value()

