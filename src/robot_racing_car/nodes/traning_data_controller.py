#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import csv
import time
import os

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Joy

class TrainingDataController:
    MOVE_FORWARD_COMMAND = 0
    MOVE_BACK_COMMAND = 1
    TURN_LEFT_COMMAND = 2
    TURN_RIGHT_COMMAND = 3
    BREAK_COMMAND = 4

    bridge = CvBridge()
    last_joystick_command = BREAK_COMMAND

    def joystick_callback(self, message):
        if (message.axes[6] == 1.0):
            self.last_joystick_command = self.TURN_LEFT_COMMAND

        if (message.axes[6] == -1.0):
            self.last_joystick_command = self.TURN_RIGHT_COMMAND

        if (message.axes[7] == 1.0):
            self.last_joystick_command = self.MOVE_FORWARD_COMMAND

        if (message.axes[7] == -1.0):
            self.last_joystick_command = self.MOVE_BACK_COMMAND

        if (message.axes[6] == -0.0 and message.axes[7] == -0.0):
            self.last_joystick_command = self.BREAK_COMMAND

    def image_callback(self, message):
        try:
	    np_arr = np.fromstring(message.data, np.uint8)
            cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    	except CvBridgeError, e:
            rospy.logerror(e)
    	else: 
            training_data_folder = '../training_data/'
 	    training_data_file_path = training_data_folder + 'training_data.csv'

            append_write = 'w'

	    if os.path.exists(training_data_file_path):
                append_write = 'a'

            image_file_postfix = time.strftime("%Y_%m_%d-%H_%M_%S_%s")
	    image_file_name = 'snapshot_' + image_file_postfix + '.jpeg'
            cv2.imwrite(training_data_folder + image_file_name, cv2_img)

            with open(training_data_file_path, append_write) as csvfile:
    	        csv_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                csv_writer.writerow([image_file_name, self.last_joystick_command])

    def __init__(self):
        rospy.init_node('camera_subscriber', log_level=rospy.DEBUG)
        rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback, queue_size = 1)

        rospy.Subscriber('joy', Joy, self.joystick_callback)

if __name__ == '__main__':
    controller = TrainingDataController()
    
    rospy.spin()
