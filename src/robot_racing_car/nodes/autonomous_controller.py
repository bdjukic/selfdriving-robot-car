#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import tensorflow as tf
import h5py
import constants.py

from std_msgs.msg import UInt8
from sensor_msgs.msg import CompressedImage
from keras.models import load_model


class AutonomousController:

    model = load_model("../model/model.h5")
    graph = tf.get_default_graph()
    last_image = None

    def camera_callback(self, message):
        np_arr = np.fromstring(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        small_image = np.empty((image.shape[1] / constants.IMAGE_SCALE, image.shape[0] / constants.IMAGE_SCALE))
        image = cv2.resize(image, small_image.shape, interpolation=cv2.INTER_LINEAR)

        self.last_image = np.asarray(image)

    def steering_callback(self, callback_argument):
        if self.last_image is None:
            return

        with self.graph.as_default():
            steering_command = self.model.predict(self.last_image[None, :, :, :], batch_size=1)
            steering_command = np.argmax(steering_command)
            rospy.logdebug("Steering command: " + str(steering_command))

            self.robot_car_steering_publisher.publish(steering_command)

            if steering_command == constants.TURN_LEFT_COMMAND or steering_command == constants.TURN_RIGHT_COMMAND:
                rospy.sleep(0.1)
                self.robot_car_steering_publisher.publish(constants.MOVE_FORWARD_COMMAND)
                rospy.sleep(0.2)

    def __init__(self):
        rospy.init_node("autonomous_controller", log_level=rospy.DEBUG)

        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.camera_callback, queue_size=1)
        self.robot_car_steering_publisher = rospy.Publisher("robot_car_steering", UInt8, queue_size=1)

        rospy.Timer(rospy.Duration(0.25), self.steering_callback)


if __name__ == "__main__":
    controller = AutonomousController()

    rospy.spin()
