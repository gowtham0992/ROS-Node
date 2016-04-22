import numpy as np
import cv2
import rospy
import rosbag
import roslib
import sys

from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

class SidewalkDetector():
	def __init__(self):
		
		self.image_pub = rospy.Publisher('/sidewalk_detector/color/image_raw',Image, queue_size=10)
		self.points_in_pub = rospy.Publisher('/sidewalk_detector/depth/points_in',PointCloud2, queue_size=10)
		self.points_out_pub = rospy.Publisher('/sidewalk_detector/depth/points_out',PointCloud2, queue_size=10)
		self.bridge = CvBridge()

	
	def get_roi(self,cv_image):
		self.h,self.w,self._ = cv_image.shape
		self.roi_w = 30
		self.roi_h = 30

		# ROI
		self.roi = cv_image[(self.h - self.roi_h):(self.h), (self.w/2.0 - self.roi_w/2.0):(self.w/2.0 + self.roi_w/2.0)]

		# Color Conversion to HSV
		self.roi_hsv = cv2.cvtColor(self.roi, cv2.COLOR_BGR2HSV)
		
		# Normalization
		self.roi_hist = cv2.calcHist( [self.roi_hsv], [0, 1], None, [9, 5], [0, 180, 0, 256] )
		self.roi_norm = cv2.normalize(self.roi_hist,self.roi_hist,0,1,cv2.NORM_MINMAX)
		return self.roi_norm


	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv_image = cv2.flip(cv_image,0)
		self.h,self.w,self._ = cv_image.shape
		self.roi_w = 30
		self.roi_h = 30

		# Rectangle for ROI
		cv2.rectangle(cv_image,(int(self.w/2.0 - self.roi_w/2.0), self.h - self.roi_h),(int(self.w/2.0 + self.roi_w/2.0), self.h),(0,0,255),3) 
		self.image = cv_image
		self.roi_norm = self.get_roi(cv_image)
		self.hsv_roi = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		self.dst = cv2.calcBackProject([self.hsv_roi],[0,1],self.roi_norm,[0,180,0,256],1)
		self.term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

		# Now convolute with circular disc
		self.disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
		cv2.filter2D(self.dst,-1,self.disc,self.dst)

		# OTSU Algorithm
		self.ret,self.thresh = cv2.threshold(self.dst,127,192,0+cv2.THRESH_OTSU)
		self.thresh_copy = self.thresh.copy()

		#Performing Erosion and Dilation
		self.kernel = np.ones((9,9),np.uint8)
		self.thresh = cv2.erode(self.thresh, self.kernel, iterations = 1)
		self.thresh = cv2.dilate(self.thresh, self.kernel, iterations=1)

		self.thresh = cv2.merge((self.thresh,self.thresh,self.thresh))
		self.roi_norm = cv2.GaussianBlur(self.dst,(5,5),0)
		self.res = cv2.bitwise_xor(self.image,self.thresh)

		# Code to draw the contours and fill them in
		self.contours, self.hierarchy = cv2.findContours(self.thresh_copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		self.cnts = sorted(self.contours, key = cv2.contourArea, reverse = True)[:5]
		cv2.drawContours(self.res, self.cnts, -1,(0,255,0),2)
		cv2.fillPoly(self.res,self.contours,(100,200,100))

		cv2.imshow('frame',self.res)
		cv2.waitKey(3)
		
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

		rospy.loginfo('callback triggered')


def main(args):
	sidewalk_detector = SidewalkDetector()
	rospy.Subscriber('camera/color/image_raw', Image, sidewalk_detector.callback)
	rospy.init_node('sidewalk_detector', anonymous=True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
