#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
from pybricks.parameters import Button, Port, Stop
from pybricks.tools import StopWatch, wait

# from pybricks.robotics import DriveBase

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the components.
left_motor = Motor(Port.A)
left_motor.control.limits(None, 400, None)
right_motor = Motor(Port.D)
right_motor.control.limits(None, 400, None)
arm_motor = Motor(Port.C)
arm_motor.control.limits(None, 250, None)
gyro = GyroSensor(Port.S2)
gyro.reset_angle(0)
color_sensor = ColorSensor(Port.S3)

# Initialize the drive base.
# robot = DriveBase(left_motor, right_motor, wheel_diameter=68.8, axle_track=127)

# line follow variables
sensor_value = 0
target_value = 50
min_value = 100
KP = 1.0

SPEED = 200  # normal speed
MIN_TOLERANCE = 6


def init_arm():
    lower_arm()
    arm_motor.reset_angle(0)

def calibrate_sensor_values():
    global target_value, min_value

    min = 100
    max = 0
    left_motor.run(150)
    right_motor.run(-150)

    for j in range(15000):
        reading = color_sensor.reflection()
        if reading < min:
            min = reading
        elif reading > max:
            max = reading

    target_value = (max + min) / 2

    min_value = max

    left_motor.stop()
    right_motor.stop()


def follow_line():  # makes robot turn to follow the line
    global sensor_value

    sensor_value = color_sensor.reflection()

    error = target_value - sensor_value

    steering_value = error * KP

    left_motor.run(SPEED + steering_value)
    right_motor.run(SPEED - steering_value)

def raise_arm():
    arm_motor.run_angle(180, Stop.HOLD, 100, True)

def lower_arm():
    arm_motor.run_until_stalled(220, Stop.COAST, 30)  # DUTY LIMIT TO CHANGE


def pick_up_cube(): # TO IMPROVE
    global sensor_value
    left_motor.run_until_stalled(280, Stop.BRAKE, 25)
    right_motor.run_until_stalled(280, Stop.BRAKE, 25)
    raise_arm()

    wait(200)

    left_motor.run_angle(-100, -10, Stop.HOLD, False)
    right_motor_motor.run_angle(-100, -10, Stop.HOLD, False)
    lower_arm()

    wait(300)

    left_motor.run(150)
    right_motor.run(-150)
    gyro.reset_angle(0)
    while sensor_value > min_value + MIN_TOLERANCE and gyro.angle() < 140:
        sensor_value = color_sensor.reflection()
        wait(5)
    left_motor.stop()
    right_motor.stop()


def turn_right():
    global sensor_value
    gyro.reset_angle(0)
    left_motor.run(SPEED)
    right_motor.hold()
    while gyro.angle() < 80 and sensor_value > min_value + MIN_TOLERANCE:
        sensor_value = color_sensor.reflection()
        wait(5)
    left_motor.stop()
    right_motor.stop()

def turn_left(): # probably won't work
    global sensor_value
    gyro.reset_angle(0)
    left_motor.hold()
    right_motor.run(SPEED)
    while gyro.angle() < 50 and sensor_value > min_value + MIN_TOLERANCE:
        sensor_value = color_sensor.reflection()
        wait(5)
    left_motor.stop()
    right_motor.stop()


#path = ['s','s','s','s','sc','s','s','rc','e'] # red cubes
path = ['s','r','l','s','e'] # test

while True:

    while not any(ev3.buttons.pressed()):
        wait(10)

    if Button.UP in ev3.buttons.pressed():
        init_arm()
        while True:
            follow_line()
            if sensor_value < min_value + MIN_TOLERANCE:  # detect intersection
                instruction = path.pop(0)
                if instruction[0] == 's': # continue straight
                    print('fat')
                elif instruction[0] == 'r': # turn right
                    turn_right()
                elif instruction[0] == 'l': # turn left
                    turn_left()
                elif instruction == 'e': # end
                    left_motor.brake()
                    right_motor.brake()
                    wait(100)
                    ev3.speaker.beep(500,6000)

                if len(instruction) > 1:
                    time = StopWatch()
                    time.reset()
                    while time.time() < 800:
                        follow_line()
                    if instruction[1] == 'c': # pick up cube
                        pick_up_cube()


    elif Button.DOWN in ev3.buttons.pressed():
        calibrate_sensor_values()