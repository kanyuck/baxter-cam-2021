#!/usr/bin/python2
import rospy
import cv2
import cv_bridge
import baxter_interface
import numpy as np
from sensor_msgs.msg import Image
from tshirt_scripts.msg import Center

class find_center:
	
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		#subscribe to right camera image
		self.image_sub = rospy.Subscriber('/cameras/right_hand_camera/image', Image, self.image_callback)
		#publish pixel location of center 
		self.pub = rospy.Publisher ("center_location", Center, queue_size=10)
		self.centerPoint = Center()

	def image_callback(self, ros_img):
		image = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_red = np.array([120,50,30])
		upper_red = np.array([180,255,255])
		red_mask = cv2.inRange(hsv, lower_red, upper_red)
		red_color = cv2.bitwise_and(image, image, mask= red_mask)
		red_crop = red_color #[200:400, :]
		gray = cv2.cvtColor(red_crop, cv2.COLOR_BGR2GRAY)
		_, threshold = cv2.threshold(gray, 20, 200, cv2.THRESH_BINARY)
		contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		
		if len(contours) > 0:
			areas = [cv2.contourArea(c) for c in contours]
			max_index = np.argmax(areas)
			largest_contour = contours[max_index]
		else:
			largest_contour = None
		cv2.drawContours(red_crop, largest_contour, -1, (0,255,0), 1)
		
		M = cv2.moments(largest_contour)
		if M["m00"] != 0:
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])
		else:
			cX, cY = 0, 0
		self.centerPoint.x = cX
		self.centerPoint.y = cY
		self.pub.publish(self.centerPoint)
		cv2.putText(red_crop, str(cX) + ', ' + str(cY), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
		cv2.imshow('red', red_crop)
		cv2.imshow('idk', image)
		cv2.waitKey(1)	
		

if __name__ == '__main__':
	rospy.init_node('find_center', anonymous=True)
	fc = find_center()
	rospy.spin()
	cv2.destroyAllWindows()

