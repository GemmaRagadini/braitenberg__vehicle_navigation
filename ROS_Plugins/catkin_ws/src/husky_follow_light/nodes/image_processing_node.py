#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageProcessingNode:

    def __init__(self):
        rospy.init_node('image_processing_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/husky_model/husky/camera', Image, self.image_callback)
        self.action_pub = rospy.Publisher('/decision', Float32, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_data = cv_image
            self.image_received = True
            decision = self.find_dominant_sector(cv_image)
            self.action_pub.publish(decision)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")
       
       
    def find_dominant_sector(self, cv_image):
        if cv_image is None:
            rospy.logerr("Errore uploading image.")
            return 'none', 0

        height, width, _ = cv_image.shape
        center_limit = width // 2
        left_region = cv_image[:, :center_limit, :]
        right_region = cv_image[:, center_limit:, :]

        red_pixels_left = self.count_red_pixels(left_region)
        red_pixels_right = self.count_red_pixels(right_region)
        pixel_values = [
            ("left", red_pixels_left),
            ("right", red_pixels_right)
            ]
        tolerance = 10
        
        # not seen red 
        if red_pixels_left < 20 and red_pixels_right < 20:
            return 1.2
        
        if abs(red_pixels_left - red_pixels_right) <= tolerance: # red is in the center
            return 0
        else:
            if red_pixels_left > red_pixels_right:
                if red_pixels_right == 0: 
                    relative_difference = 1
                else:
                    relative_difference = ((red_pixels_left - red_pixels_right) / red_pixels_left) 
            else:
                if red_pixels_left == 0:
                    relative_difference = -1
                else:
                    relative_difference = -((red_pixels_right - red_pixels_left) / red_pixels_right) 


        return relative_difference



    def count_red_pixels(self, image):
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 90, 0])
        upper_red1 = np.array([10, 255, 50])

        lower_red2 = np.array([170, 90, 0])
        upper_red2 = np.array([179, 255, 50])

        mask_red1 = cv2.inRange(image_hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(image_hsv, lower_red2, upper_red2)
        mask_red = mask_red1 | mask_red2

        red_pixels = np.count_nonzero(mask_red)
        
        return red_pixels



if __name__ == '__main__':
    try:
        node = ImageProcessingNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
