#!/usr/bin/env python

import rospy
import constants.py

from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy


class RobotCarController:
    def joystick_callback(self, message):
        if message.axes[6] == 1.0:
            self.robot_car_publisher.publish(constants.TURN_LEFT_COMMAND)
            rospy.logdebug("Turn left")
        elif message.axes[6] == -1.0:
            self.robot_car_publisher.publish(constants.TURN_RIGHT_COMMAND)
            rospy.logdebug("Turn right")
        elif message.axes[7] == 1.0:
            self.robot_car_publisher.publish(constants.MOVE_FORWARD_COMMAND)
            rospy.logdebug("Move forward")
        elif message.axes[7] == -1.0:
            self.robot_car_publisher.publish(constants.MOVE_BACK_COMMAND)
            rospy.logdebug("Move back")
        elif message.axes[6] == -0.0 and message.axes[7] == -0.0:
            self.robot_car_publisher.publish(constants.BREAK_COMMAND)
            rospy.logdebug("Break")

    def __init__(self):
        rospy.Subscriber("joy", Joy, self.joystick_callback)
        self.robot_car_publisher = rospy.Publisher("robot_car_steering", UInt8, queue_size=10)

if __name__ == "__main__":
    rospy.init_node("robot_car_controller", log_level=rospy.DEBUG)

    controller = RobotCarController()

    rospy.spin()
