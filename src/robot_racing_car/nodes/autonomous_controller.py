#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from std_msgs.msg import UInt8

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from keras.models import load_model
import h5py

class AutonomousController:
    bridge = CvBridge()
    model = load_model('../model/model.h5')

    def image_callback(self, message):
        try:
	    np_arr = np.fromstring(message.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    	except CvBridgeError, e:
            rospy.logerror(e)
    	else: 
	    image_array = np.asarray(image)
            steering_command = int(model.predict(image_array[None, :, :, :], batch_size=1))
	    self.robot_car_steering_publisher.publish(UInt8(steering_command))

    def __init__(self):
        rospy.init_node('camera_subscriber', log_level=rospy.DEBUG)
        rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback, queue_size = 1)

        self.robot_car_steering_publisher = rospy.Publisher('robot_car_steering', UInt8, queue_size=10)

if __name__ == '__main__':
    controller = AutonomousController()
    
    rospy.spin()
