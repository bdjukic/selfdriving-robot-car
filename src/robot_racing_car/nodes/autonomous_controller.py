#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import tensorflow as tf
import h5py
import constants

from std_msgs.msg import UInt8
from sensor_msgs.msg import CompressedImage
from keras.models import load_model
from random import uniform


class AutonomousController:

    model = load_model("../model/model.h5")
    graph = tf.get_default_graph()
    last_image = None
    is_steering = False

    def camera_callback(self, message):
        np_arr = np.fromstring(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        small_image = np.empty((image.shape[1] / constants.IMAGE_SCALE, image.shape[0] / constants.IMAGE_SCALE))
        image = cv2.resize(image, small_image.shape, interpolation=cv2.INTER_LINEAR)

        self.last_image = np.asarray(image)

        with self.graph.as_default():
            self.last_image = self.last_image.reshape((1,) + self.last_image.shape)
            prediction = self.model.predict(self.last_image)

            steering_command = np.argmax(prediction[0][0])
            throttle_value = prediction[1][0][0]

            rospy.logdebug("Predicted steering command: " + str(prediction[0][0]))
            rospy.logdebug("Predicted throttle: " + str(throttle_value))

            self.robot_car_steering_speed_publisher.publish(int(throttle_value))

            if steering_command != constants.MOVE_FORWARD_COMMAND:
                is_steering = True
                self.robot_car_steering_publisher.publish(steering_command)
                is_steering = False

    def steering_callback(self, callback_argument):
        if not self.is_steering:
            self.robot_car_steering_publisher.publish(constants.MOVE_FORWARD_COMMAND)

    def __init__(self):
        rospy.init_node("autonomous_controller", log_level=rospy.DEBUG)

        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.camera_callback, queue_size=1)

        self.robot_car_steering_publisher = rospy.Publisher("robot_car_steering", UInt8, queue_size=1)
        self.robot_car_steering_speed_publisher = rospy.Publisher("robot_car_speed", UInt8, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.steering_callback)


if __name__ == "__main__":
    controller = AutonomousController()

    rospy.spin()
