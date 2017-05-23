#!/usr/bin/env python

import rospy
import constants

from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy


class RobotCarController:
    current_robot_car_speed = constants.DEFAULT_SPEED

    def joystick_steering_callback(self, message):
        # Robot steering control
        if message.axes[6] == 1.0:
            self.robot_car_steering_publisher.publish(constants.TURN_LEFT_COMMAND)
            rospy.logdebug("Turn left")
        elif message.axes[6] == -1.0:
            self.robot_car_steering_publisher.publish(constants.TURN_RIGHT_COMMAND)
            rospy.logdebug("Turn right")
        elif message.axes[7] == 1.0:
            self.robot_car_steering_publisher.publish(constants.MOVE_FORWARD_COMMAND)
            rospy.logdebug("Move forward")
        elif message.axes[7] == -1.0:
            self.robot_car_steering_publisher.publish(constants.MOVE_BACK_COMMAND)
            rospy.logdebug("Move back")
        elif message.axes[6] == -0.0 and message.axes[7] == -0.0 and message.axes[5] == 1.0:
            self.robot_car_steering_publisher.publish(constants.BREAK_COMMAND)
            self.current_robot_car_speed = 100
            self.robot_car_steering_speed_publisher.publish(self.current_robot_car_speed)
            rospy.logdebug("Break")

    # Robot throttle control
    def joystick_throttle_callback(self, message):
        if message.axes[5] != 1.0:
            normalized = -message.axes[5] + 1
            speed_ratio = normalized / 2
            new_speed = 155 * speed_ratio + constants.DEFAULT_SPEED

            self.current_robot_car_speed = new_speed
        else:
            self.current_robot_car_speed = constants.DEFAULT_SPEED

        self.robot_car_steering_speed_publisher.publish(self.current_robot_car_speed)

        if message.buttons[0] == 1.0:
            # A button pressed
            delta_decrease = 10

            if self.current_robot_car_speed - delta_decrease < 0:
                return

            self.current_robot_car_speed -= 10
            self.robot_car_steering_speed_publisher.publish(self.current_robot_car_speed)
            rospy.logdebug("Speed decreased to: " + str(self.current_robot_car_speed))
        elif message.buttons[1] == 1.0:
            # B button pressed
            delta_increase = 10

            if self.current_robot_car_speed + delta_increase > 255:
                return

            self.current_robot_car_speed += delta_increase
            self.robot_car_steering_speed_publisher.publish(self.current_robot_car_speed)
            rospy.logdebug("Speed increased to: " + str(self.current_robot_car_speed))
        elif message.buttons[2] == 1.0:
            # X button pressed
            self.current_robot_car_speed = self.DEFAULT_SPEED
            self.robot_car_steering_speed_publisher.publish(self.current_robot_car_speed)
            rospy.logdebug("Speed set to default: " + str(self.current_robot_car_speed))
        elif message.buttons[3] == 1.0:
            # Y button pressed
            self.current_robot_car_speed = 255
            self.robot_car_steering_speed_publisher.publish(self.current_robot_car_speed)
            rospy.logdebug("Speed set to maximum: " + str(self.current_robot_car_speed))

    def __init__(self):
        rospy.Subscriber("joy", Joy, self.joystick_steering_callback)
        rospy.Subscriber("joy", Joy, self.joystick_throttle_callback)

        self.robot_car_steering_publisher = rospy.Publisher("robot_car_steering", UInt8, queue_size=1)
        self.robot_car_steering_speed_publisher = rospy.Publisher("robot_car_speed", UInt8, queue_size=1)


if __name__ == "__main__":
    rospy.init_node("robot_car_controller", log_level=rospy.DEBUG)

    controller = RobotCarController()

    rospy.spin()
