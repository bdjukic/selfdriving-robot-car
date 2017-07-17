#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import tensorflow as tf
import h5py

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from keras.models import load_model
from random import uniform


class AutopilotController:


    IMAGE_SCALE = 4
    
    model = load_model("./model/model.h5")
    graph = tf.get_default_graph()
            	
    def linear_unbin(self, value_to_unbin):
        unbinned_value = value_to_unbin * (2.0 / 14.0) - 1

    	return unbinned_value
    
    def unbin_matrix(self, matrix_to_unbin):
        unbinned_matrix=[]
        
        for value_to_unbin in matrix_to_unbin:
            unbinned_value = np.argmax(value_to_unbin)
            unbinned_value = self.linear_unbin(unbinned_value)
            unbinned_matrix.append(unbinned_value)

	    return np.array(unbinned_matrix)

    def camera_callback(self, message):
        np_arr = np.fromstring(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        small_image = np.empty((image.shape[1] / constants.IMAGE_SCALE, image.shape[0] / constants.IMAGE_SCALE))
        image = cv2.resize(image, small_image.shape, interpolation=cv2.INTER_LINEAR)

        last_image = np.asarray(image)

        with self.graph.as_default():
            last_image = last_image.reshape((1,) + last_image.shape)
            steering_angle_prediction = self.model.predict(last_image)
            
            # Grouping data into bins: https://msdn.microsoft.com/en-us/library/azure/dn913065.aspx
            steering_angle = self.unbin_matrix(steering_angle_prediction)
            
            throttle = 0.175
            
            #rospy.logdebug("Throttle: " + str(throttle))
            
            command = str(steering_angle[0]) + ":" + str(throttle)
            rospy.logdebug("Steering angle: " + command)
			
            self.autopilot_publisher.publish(command)

    def __init__(self):
        rospy.init_node("autopilot_controller", log_level=rospy.DEBUG)
        
        self.autopilot_publisher = rospy.Publisher("autopilot", String, queue_size=1)

        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.camera_callback, queue_size=1)


if __name__ == "__main__":
    controller = AutopilotController()

    rospy.spin()
