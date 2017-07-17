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
        small_image = np.empty((image.shape[1] / self.IMAGE_SCALE, image.shape[0] / self.IMAGE_SCALE))
        image = cv2.resize(image, small_image.shape, interpolation=cv2.INTER_LINEAR)

        last_image = np.asarray(image)

        with self.graph.as_default():
            last_image = last_image.reshape((1,) + last_image.shape)
            
            # Doing steering angle and throttle prediction based on image
            prediction = self.model.predict(last_image)
            
            # Grouping data into bins: https://msdn.microsoft.com/en-us/library/azure/dn913065.aspx
            steering_angle = self.unbin_matrix(prediction)[0]
            throttle = 0.19
            
            command = str(steering_angle) + ":" + str(throttle)

            rospy.logdebug("Autopilot command: " + command)
			
            self.autopilot_publisher.publish(command)
            
    def shutdownCallback(self):
    	# On node shutdown set the steering angle and throttle to 0. This message is not guaranteed to be published. 
    	
    	rospy.logdebug("Setting steering angle and throttle to 0")
    	self.autopilot_publisher.publish("0:0")

    def __init__(self):
        rospy.init_node("autopilot_controller", log_level=rospy.DEBUG)
        
        self.autopilot_publisher = rospy.Publisher("autopilot", String, queue_size=1)

        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.camera_callback, queue_size=1)
        
        rospy.on_shutdown(self.shutdownCallback)


if __name__ == "__main__":
    controller = AutopilotController()

    rospy.spin()
