from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
gyro = GyroSensor(Port.S1)
color_sensor = ColorSensor(Port.S2)

# Initialize the drive base.
# robot = DriveBase(left_motor, right_motor, wheel_diameter=68.8, axle_track=127)

gyro.reset_angle(0)

# line follow variables
target_value = 0
kp = 1.0
kd = 0.0
prev_error = 0

# 
speed = 50 # normal straight line speed


def calibrateTargetValue():
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


def followLine(): # makes robot steer to follow the line
    sensor_value = color_sensor.reflection()

    error = target_value - sensor_value

    derivative = error - prev_error

    steering_value = error * kp + derivative * kd

    left_motor.run(speed + steering_value)
    right_motor.run(speed - steering_value)

    prev_error = error


calibrateTargetValue()

while True:
    followLine()
