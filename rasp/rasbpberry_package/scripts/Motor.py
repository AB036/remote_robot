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
    def __init__(self, pin):
        RPi.GPIO.setmode(RPi.GPIO.BCM)
        if not isinstance(pin, int):
            raise ValueError("pin pin should be int")

        self.__pin = pin

        # Setup GPIO as output on raspberry
        RPi.GPIO.setup(self.__pin, RPi.GPIO.OUT)

        # Set the speed to 0 at starting
        self.__pwm = RPi.GPIO.PWM(self.__pin, 50)
        self.__pwm.start(0)

    def move_speed(self, speed):
        MAX_PWM = 14
        MIN_PWM = 0
        if not isinstance(speed, float):
            raise ValueError("Speed should be float")
        if not (speed > -100 and speed < 100):
            raise ValueError("Speed is a signed percentage and should be between -100 and 100")
        if speed != 0:
            """setup the pins to turn motor in positive direction"""
            #print("motor move speed negative")
            self.__pwm.ChangeDutyCycle((MIN_PWM+MAX_PWM)/2 - (speed / 100) * (MAX_PWM - MIN_PWM) / 2)
        elif speed > 0:
            """setup the pins to turn motor in negative direction"""
            #print("motor move speed positive")
            print((MIN_PWM + MAX_PWM) / 2 - (speed / 100) * (MAX_PWM - MIN_PWM) / 2)
            self.__pwm.ChangeDutyCycle((MIN_PWM+MAX_PWM)/2 - (speed / 100) * (MAX_PWM - MIN_PWM) / 2)
        else:
            """setup the pins to stop motor """
            #print("motor move speed stop")
            self.__pwm.ChangeDutyCycle(0)

