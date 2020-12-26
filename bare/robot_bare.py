from Adafruit_MotorHAT import Adafruit_MotorHAT
import time


def run(motor_driver, channel, value):

    if(channel == 1):
        _ina = 1
        _inb = 0
    else:
        _ina = 2
        _inb = 3

    mapped_value = int(255.0 * value)
    speed = min(max(abs(mapped_value), 0), 255)

    _motor = motor_driver.getMotor(channel)
    _motor.setSpeed(speed)

    if mapped_value < 0:
        _motor.run(Adafruit_MotorHAT.FORWARD)
        motor_driver._pwm.setPWM(_ina, 0, 0)
        motor_driver._pwm.setPWM(_inb, 0, speed*16)
    else:
        _motor.run(Adafruit_MotorHAT.BACKWARD)
        motor_driver._pwm.setPWM(_ina, 0, speed*16)
        motor_driver._pwm.setPWM(_inb, 0, 0)


if __name__ == "__main__":
    i2c_bus = 1
    left_motor_channel = 1
    right_motor_channel = 2

    motor_driver = Adafruit_MotorHAT(i2c_bus=i2c_bus)

    run(motor_driver, left_motor_channel, -0.1)
    time.sleep(0.5)
    run(motor_driver, left_motor_channel, 0)

    run(motor_driver, right_motor_channel, 0.1)
    time.sleep(0.5)
    run(motor_driver, right_motor_channel, 0)
