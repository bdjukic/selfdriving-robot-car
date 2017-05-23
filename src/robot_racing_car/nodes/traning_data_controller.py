#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import csv
import time
import os
import constants

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Joy
from message_filters import ApproximateTimeSynchronizer, Subscriber


class TrainingDataController:
    def approx_time_synchronizer_callback(self, image_message, joystick_message):
        if joystick_message.axes[6] == -0.0 and joystick_message.axes[7] == -0.0:
            # Ignore BREAK_COMMAND while getting training data
            return

        if joystick_message.axes[6] == 1.0:
            last_joystick_command = constants.TURN_LEFT_COMMAND
        elif joystick_message.axes[6] == -1.0:
            last_joystick_command = constants.TURN_RIGHT_COMMAND
        elif joystick_message.axes[7] == 1.0:
            last_joystick_command = constants.MOVE_FORWARD_COMMAND
        elif joystick_message.axes[7] == -1.0:
            last_joystick_command = constants.MOVE_BACK_COMMAND

        current_robot_car_speed = constants.DEFAULT_SPEED

        if joystick_message.axes[5] != 1.0:
            normalized = -joystick_message.axes[5] + 1
            speed_ratio = normalized / 2
            current_robot_car_speed = 155 * speed_ratio + constants.DEFAULT_SPEED

        np_arr = np.fromstring(image_message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        small_image = np.empty((image.shape[1] / constants.IMAGE_SCALE, image.shape[0] / constants.IMAGE_SCALE))
        last_image = cv2.resize(image, small_image.shape, interpolation=cv2.INTER_LINEAR)

        training_data_folder = "../training_data/"
        training_data_file_path = training_data_folder + "training_data.csv"

        append_write_flag = "w"

        if os.path.exists(training_data_file_path):
            append_write_flag = "a"

        image_file_postfix = str(time.time())
        image_file_name = "snapshot_" + image_file_postfix + ".jpeg"

        cv2.imwrite(training_data_folder + image_file_name, last_image)

        with open(training_data_file_path, append_write_flag) as csvfile:
            csv_writer = csv.writer(csvfile, delimiter=" ", quotechar="|", quoting=csv.QUOTE_MINIMAL)
            csv_writer.writerow([image_file_name, last_joystick_command, current_robot_car_speed])

    def __init__(self):
        rospy.init_node("training_data_controller", log_level=rospy.DEBUG)

        camera_node_subscriber = Subscriber("/raspicam_node/image/compressed", CompressedImage)
        joystick_subscriber = Subscriber("joy", Joy)

        approx_time_synchronizer = ApproximateTimeSynchronizer([camera_node_subscriber, joystick_subscriber], 10, 1)
        approx_time_synchronizer.registerCallback(self.approx_time_synchronizer_callback)

if __name__ == "__main__":
    controller = TrainingDataController()

    rospy.spin()
