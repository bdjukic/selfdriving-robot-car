#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy

class RobotCarController:

    MOVE_FORWARD_COMMAND = 0
    MOVE_BACK_COMMAND = 1
    TURN_LEFT_COMMAND = 2
    TURN_RIGHT_COMMAND = 3
    BREAK_COMMAND = 4

    def joystick_callback(self, message):
        if (message.axes[6] == 1.0):
            self.robot_car_publisher.publish(UInt8(self.TURN_LEFT_COMMAND))
            rospy.logdebug('Turn left')

        if (message.axes[6] == -1.0):
            self.robot_car_publisher.publish(UInt8(self.TURN_RIGHT_COMMAND))
            rospy.logdebug('Turn right')

        if (message.axes[7] == 1.0):
            self.robot_car_publisher.publish(UInt8(self.MOVE_FORWARD_COMMAND))
            rospy.logdebug('Move forward')

        if (message.axes[7] == -1.0):
            self.robot_car_publisher.publish(UInt8(self.MOVE_BACK_COMMAND))
            rospy.logdebug('Move back')

        if (message.axes[6] == -0.0 and message.axes[7] == -0.0):
            self.robot_car_publisher.publish(UInt8(self.BREAK_COMMAND))
            rospy.logdebug('Break')

    def __init__(self):
	# Create Joystick node subscriber
        rospy.Subscriber('joy', Joy, self.joystick_callback)

        # Create Arduino node publisher
        self.robot_car_publisher = rospy.Publisher('robot_car_steering', UInt8, queue_size=10)

if __name__ == '__main__':
    rospy.init_node('robot_car_controller', log_level=rospy.DEBUG)

    controller = RobotCarController()
    
    rospy.spin()
