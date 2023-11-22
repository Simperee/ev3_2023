#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Button
from pybricks.robotics import DriveBase

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the components.
left_motor = Motor(Port.A)
left_motor.control.limits(None, 500, None)
right_motor = Motor(Port.D)
right_motor.control.limits(None, 250, None)
arm_motor = Motor(Port.C)
arm_motor.control.limits(None,500,None)
gyro = GyroSensor(Port.S2)
gyro.reset_angle(0)
color_sensor = ColorSensor(Port.S3)

robot = DriveBase(left_motor, right_motor, wheel_diameter=68.8, axle_track=127)

# line follow variables
max_value = 100
target_value = 50
KP = 1.5
KD = 0.5
KI = 0.0
prev_error = 0
integral = 0

SPEED = 200 # normal straight line speed

def calibrateTargetValue():
    global target_value, max_value

    min = 100
    max = 0
    sum_min = 0
    sum_max = 0
    left_motor.run(360)
    right_motor.run(-360)

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
    max_value = avg_max
    left_motor.stop()
    right_motor.stop()

    target_value = (avg_min + avg_max) / 2

def path():
    dist_travelled = 0
    while robot.distance()<280:
        follow_line()
    if robot.distance()>=280:
        left_motor.stop()
        right_motor.stop()
        dist_travelled+=280
        print(dist_travelled)
        robot.reset()

def follow_line(): # makes robot turn to follow the line
     global prev_error, integral

     sensor_value = color_sensor.reflection()

     if sensor_value > max_value - 4.0:
         left_motor.run(-360)
         right_motor.run(360)
         return

     error = target_value - sensor_value

     derivative = error - prev_error

     integral += error

   
     steering_value = error * KP + derivative * KD + integral * KI
     left_motor.run(SPEED+steering_value)
     right_motor.run(SPEED-steering_value)

#     prev_error = error
    


while True:
    if Button.DOWN in EV3Brick.buttons.pressed():
        calibrateTargetValue()

    if Button.UP in EV3Brick.buttons.pressed():
        while True:
            path()