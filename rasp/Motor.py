import sys
import time


class Motor:
    def __init__(self):
        pass

    def move_speed(self, speed):
        raise NotImplementedError()


class VirtualMotor(Motor):
    def __init__(self, name):
        self.__name = name

    def move_speed(self, speed):
        if not isinstance(speed, float):
            raise ValueError("Speed should be float")
        if not (-100 < speed < 100):
            raise ValueError("Speed is a signed percentage and should be between -100 and 100")
        if speed < 0:
            print("Motor " + self.__name + " turn reverse direction with speed " + str(-speed))
        elif speed > 0:
            print("Motor " + self.__name + " turn normal direction with speed " + str(speed))
        else:
            print("Motor " + self.__name + " is stopped")


class TrueMotor(Motor):
    def __init__(self, pin_in_1, pin_in_2, pin_pwm):
        import RPi.GPIO as GPIO
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
        GPIO.setup(self.__pin_in_1, GPIO.OUT)
        GPIO.setup(self.__pin_in_2, GPIO.OUT)
        GPIO.setup(self.__pin_pwm, GPIO.OUT)

        # Set the speed to 0 at starting
        GPIO.output(self.__pin_in_1, GPIO.LOW)
        GPIO.output(self.__pin_in_2, GPIO.LOW)
        self._pwm = GPIO.PWM(self.__pin_pwm, 2000)
        self._pwm.start(0)

    def move_speed(self, speed):
        if not isinstance(float, speed):
            raise ValueError("Speed should be float")
        if not (speed > -100 and speed < 100):
            raise ValueError("Speed is a signed percentage and should be between -100 and 100")
        if speed < 0:
            GPIO.output(self.__pin_in_1, GPIO.LOW)
            GPIO.output(self.__pin_in_2, GPIO.HIGH)
            self._pwm.ChangeDutyCycle(-speed)
        elif speed > 0:
            GPIO.output(self.__pin_in_1, GPIO.HIGH)
            GPIO.output(self.__pin_in_2, GPIO.LOW)
            self._pwm.ChangeDutyCycle(speed)
        else:
            GPIO.output(self.__pin_in_1, GPIO.LOW)
            GPIO.output(self.__pin_in_2, GPIO.LOW)
            self._pwm.ChangeDutyCycle(speed)
