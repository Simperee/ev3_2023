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

    motor_left.run(speed + steeringValue)
    motor_right.run(speed - steeringValue)

    #wait(1)

targetValue = calibrateColorSensor()

while(true){
    followLine()
}
