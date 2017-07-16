#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import tensorflow as tf
import h5py
import constants

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from keras.models import load_model
from random import uniform


class AutopilotController:

    model = load_model("../model/model.h5")
    graph = tf.get_default_graph()
    
    def linear_unbin(self, b):
        a = b * (2.0/14.0) - 1
    	return a
    	
    def unbin_Y(self, Y):
        d=[]
        for y in Y:
            v = np.argmax(y)
            v = self.linear_unbin(v)
            d.append(v)
	    return np.array(d)

    def camera_callback(self, message):
        np_arr = np.fromstring(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        small_image = np.empty((image.shape[1] / constants.IMAGE_SCALE, image.shape[0] / constants.IMAGE_SCALE))
        image = cv2.resize(image, small_image.shape, interpolation=cv2.INTER_LINEAR)

        last_image = np.asarray(image)

        with self.graph.as_default():
            last_image = last_image.reshape((1,) + last_image.shape)
            steering_angle_prediction = self.model.predict(last_image)
            steering_angle = self.unbin_Y(steering_angle_prediction)
            
            throttle = 0.3

            rospy.logdebug("Steering angle: " + str(steering_angle))
            #rospy.logdebug("Throttle: " + str(throttle))
			
            self.autopilot_publisher.publish(str(steering_angle) + ":" + str(throttle))

    def __init__(self):
        rospy.init_node("autopilot_controller", log_level=rospy.DEBUG)
        
        self.autopilot_publisher = rospy.Publisher("autopilot", String, queue_size=1)

        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.camera_callback, queue_size=1)


if __name__ == "__main__":
    controller = AutopilotController()

    rospy.spin()
