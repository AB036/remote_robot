import Motor
import Robot
import time

"""Package for debug only. It tests the motors"""


class DebugRosNode:
    """ Debug rosnode. For debugging only"""
    def __init__(self):
        self.__last_time = time.time()

    def reset(self):
        self.__last_time = time.time()

    def __get_last_time(self):
        return self.__last_time

    last_time = property(__get_last_time)


left_motor = Motor.TrueMotor(17)
right_motor = Motor.TrueMotor(27)
ros_node = DebugRosNode()
robot = Robot.Robot(left_motor, right_motor, ros_node)

while 1:
    command = raw_input("Type command: ")
    print(command)
    last_time = time.time()
    ros_node.reset()
    while time.time()-last_time < 0.7:
        robot.process_command(command)
        time.sleep(0.1)


