#!/usr/bin/env python

import rospy
import Adafruit_PCA9685

from std_msgs.msg import UInt8

class SteeringController:
    LEFT_ANGLE = 1
    RIGHT_ANGLE = -1

    LEFT_PULSE = 460
    RIGHT_PULSE = 260

    def steering_callback(self, message):
        self.update(message.data)

    def map_range(x, X_min, X_max, Y_min, Y_max):
        # Linear mapping between two ranges of values
        X_range = X_max - X_min
        Y_range = Y_max - Y_min
        XY_ratio = X_range / Y_range

        y = ((x - X_min) / XY_ratio + Y_min) // 1

        return int(y)

    def update(self, angle):
        pulse = self.map_range(angle, self.RIGHT_ANGLE, self.LEFT_ANGLE, self.RIGHT_PULSE, self.LEFT_PULSE)

        self.steering_actuator.set_pwm(self.actuator_channel, 0, pulse)

    def __init__(self):
        self.steering_actuator = Adafruit_PCA9685.PCA9685()
        self.steering_actuator.set_pwm_freq(60)
        self.actuator_channel = 1

        rospy.Subscriber("robot_car_steering", UInt8, self.steering_callback)

if __name__ == "__main__":
    rospy.init_node("steering_controller", log_level=rospy.DEBUG)

    controller = SteeringController()

    rospy.spin()