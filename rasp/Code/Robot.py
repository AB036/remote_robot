import Motor
import time


class Robot:
    """This class controls the motors of the robot in function of the command"""

    def __init__(self, left_motor, right_motor, ros_node):
        if not isinstance(left_motor, Motor.Motor):
            raise ValueError("motor should be Motor Objects")
        if not isinstance(right_motor, Motor.Motor):
            raise ValueError("motor should be Motor Objects")
        self.__left_motor = left_motor
        self.__right_motor = right_motor
        self.__ros_node = ros_node
        self.__straight_speed = 95.0  # Speed for straight movement. Between 0 and 99
        self.__turning_speed = 95.0  # Speed for turning. Between 0 and 99

    def process_command(self, command):
        """Process the command : should be called frequently (10Hz)"""
        if time.time() - self.__ros_node.last_time < 0.5:  # After one second, robot stops, waiting for a new command
            if command == "up":
                print("Move up")
                self.__move_forward()
            elif command == "down":
                self.__move_backward()
            elif command == "left":
                self.__turn_left()
            elif command == "right":
                self.__turn_right()
            elif command == "stop":
                self.__stop()
            else:
                self.__stop()
        else:
            self.__stop()

    def __move_forward(self):
        self.__left_motor.move_speed(self.__straight_speed)
        self.__right_motor.move_speed(-self.__straight_speed)

    def __move_backward(self):
        self.__left_motor.move_speed(-self.__straight_speed)
        self.__right_motor.move_speed(self.__straight_speed)

    def __turn_left(self):
        self.__left_motor.move_speed(self.__turning_speed)
        self.__right_motor.move_speed(self.__turning_speed)

    def __turn_right(self):
        self.__left_motor.move_speed(-self.__turning_speed)
        self.__right_motor.move_speed(-self.__turning_speed)

    def __stop(self):
        self.__left_motor.move_speed(0.0)
        self.__right_motor.move_speed(0.0)
