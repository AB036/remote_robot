import sys
import time
import RPi.GPIO


class Motor:
    """Class representing a motor of the robot"""

    def __init__(self):
        pass

    def move_speed(self, speed):
        raise NotImplementedError()


class VirtualMotor(Motor):
    """A virtual motor is used to test the ros communication without really moving the motors (for debug only)"""
    def __init__(self, name):
        self.__name = name

    def move_speed(self, speed):
        if not isinstance(speed, float):
            raise ValueError("Speed should be float")
        if not (-100 <= speed <= 100):
            raise ValueError("Speed is a signed percentage and should be between -100 and 100")
        if speed < 0:
            print("Motor " + self.__name + " turn reverse direction with speed " + str(-speed))
        elif speed > 0:
            print("Motor " + self.__name + " turn normal direction with speed " + str(speed))
        else:
            print("Motor " + self.__name + " is stopped")


class TrueMotor(Motor):
    def __init__(self, pin_in_1, pin_in_2, pin_pwm):
        RPi.GPIO.setmode(RPi.GPIO.BCM)
        if not isinstance(pin_in_1, int):
            raise ValueError("pin_in_1 pin should be int")
        if not isinstance(pin_in_2, int):
            raise ValueError("pin_in_2 should be int")
        if not isinstance(pin_pwm, int):
            raise ValueError("pin_pwm should be int")

        self.__pin_in_1 = pin_in_1
        self.__pin_in_2 = pin_in_2
        self.__pin_pwm = pin_pwm

        # Setup GPIO as output on raspberry
        RPi.GPIO.setup(self.__pin_in_1, RPi.GPIO.OUT)
        RPi.GPIO.setup(self.__pin_in_2, RPi.GPIO.OUT)
        RPi.GPIO.setup(self.__pin_pwm, RPi.GPIO.OUT)

        # Set the speed to 0 at starting
        RPi.GPIO.output(self.__pin_in_1, RPi.GPIO.LOW)
        RPi.GPIO.output(self.__pin_in_2, RPi.GPIO.LOW)
        self._pwm = RPi.GPIO.PWM(self.__pin_pwm, 10000)
        self._pwm.start(0)

    def move_speed(self, speed):
        if not isinstance(speed, float):
            raise ValueError("Speed should be float")
        if not (speed > -100 and speed < 100):
            raise ValueError("Speed is a signed percentage and should be between -100 and 100")
        if speed < 0:
            """setup the pins to turn motor in positive direction"""
            RPi.GPIO.output(self.__pin_in_1, RPi.GPIO.LOW)
            RPi.GPIO.output(self.__pin_in_2, RPi.GPIO.HIGH)
            self._pwm.ChangeDutyCycle(-speed)
        elif speed > 0:
            """setup the pins to turn motor in negative direction"""
            RPi.GPIO.output(self.__pin_in_1, RPi.GPIO.HIGH)
            RPi.GPIO.output(self.__pin_in_2, RPi.GPIO.LOW)
            self._pwm.ChangeDutyCycle(speed)
        else:
            """setup the pins to stop motor """
            RPi.GPIO.output(self.__pin_in_1, RPi.GPIO.LOW)
            RPi.GPIO.output(self.__pin_in_2, RPi.GPIO.LOW)
            self._pwm.ChangeDutyCycle(speed)
