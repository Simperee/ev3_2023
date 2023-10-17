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
robot = DriveBase(left_motor, right_motor, wheel_diameter=68.8, axle_track=127)

gyro.reset_angle(0)

kp = 1.0 #proportional constant
speed = 50


def calibrateColorSensor():

    min = 100
    max = 0

    left_motor.run(50)
    right_motor.run(-50)

    for i in range(5000):
        reading = color_sensor.reflection()

        if reading < min:
            min = reading
        elif reading > max:
            max = reading
        
    left_motor.stop()
    right_motor.stop()

    return (max + min) / 2


def followLine():

    sensorValue = color_sensor.reflection()

    error = targetValue - sensorValue

    steeringValue = error * kp

    left_motor.run(speed + steeringValue)
    right_motor.run(speed - steeringValue)

    #wait(1)


targetValue = calibrateColorSensor()

while True:
    followLine()
