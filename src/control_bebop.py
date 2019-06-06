#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('drone')
import sys
import rospy
import numpy as np
import cv2
import time

from std_msgs.msg import String, Empty, Int32
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from getkey import getkeys, key
from drone.msg import img_obj_info

#*********************OBJECT DETECTION BY COLOR**********************************************
class image_convert:

	def __init__(self):
		#===============================================
		#	CHANGE TOPICS NECESARY FOR THE DRONE
		#===============================================
		
		area = None
		self.centerx = None
		self.centery = None
		self.image_pub = rospy.Publisher("/image_converter/output_video",Image,queue_size = 10)
		self.image_pub_res = rospy.Publisher("/object_detection/img_result", Image, queue_size=10)
		self.objInfo = rospy.Publisher("/ObjInfo", img_obj_info, queue_size=10)
		self.sub_objInfo = rospy.Subscriber("/ObjInfo", img_obj_info, self.callbackObjInfo)
		
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/bebop/image_raw",Image,self.callback)
		#self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)		#To use usb cam of computer
			
	def callbackObjInfo(self,data):
		#Data is a custom type of message created so that the area a centroid of the 
		#object can be used by the drone_cmds class.
		self.objectData(data)
		
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
 		contours, mask_image = self.HSVObjectDetection(cv_image)
		
		for cnt in contours:
			# Find the bounding box of the detected object
			# xp, yp are the coordinates of the upper-left corner of the bounding box
			# w, h are the width and height of the rectangle
			xp,yp,w,h = cv2.boundingRect(cnt) 
			
			cv2.rectangle(cv_image,(xp,yp),(xp+w,yp+h),[0,255,255], 2)
			centerx, centery = xp+w/2, yp+h/2

		cv2.imshow("Original Image", cv_image)
		cv2.waitKey(3)
		
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)
					
		
	def HSVObjectDetection(self,cv_image, toPrint = True):
		# Function that converts the image to the HSV color space and thresholds it
		# according to the patameters specified
		hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		
		# Define HSV color range
		# CHECK SEL_COLOR.PY
		lower_follow = np.array([000,180,129])		# Define lower pixel intensities to follow
		upper_follow = np.array([012,255,255])		# Define upper pixel intensities to follow
		
		lower_color_obstacle = np.array([000,180,129])
		upper_color_obstacle = np.array([012,255,255])
		
		
		# Threshold of the HSV image
		mask = cv2.inRange(hsv_image, lower_follow, upper_follow)   
		mask_eroded = cv2.erode(mask, None, iterations = 2)  
		mask_eroded_dilated = cv2.dilate(mask_eroded, None, iterations = 10) 
    
		#if toPrint:
			#print ('hsv'), hsv_image[240][320] # the center point hsv
			
		self.showImage(cv_image, mask_eroded, mask_eroded_dilated)
		# Get the number of contours detected and store it in a list
		_,contours, _ = cv2.findContours(mask_eroded_dilated,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		
		Nx = 1		# Object to take the average of centroid in case of multiple objects recognnized in only one object
		cx1 = 0
		cy1 = 0
		cx_avg = 0
		cy_avg = 0
		
		N = len(contours)		# Actual number of contours detected
		#print N
		
		if N == 0:
			area = 0
		
		for i in contours[0:Nx]:			
			index = contours.index(i)
			cnt = contours[index]
			#print cnt
			M = cv2.moments(cnt)
			#print M
			cx = int(M['m10']/M['m00'])		# X coordinate of the centroid
			cy = int(M['m01']/M['m00'])		# Y coordinate of the centroid
			cx1 += cx
			cy1 += cy

		cx_avg = cx1/Nx
		cy_avg = cy1/Nx
		
		#Key=cv2.waitKey(3)
		
		# Get the area inside de contours detected
		for i in range(N):
			area = cv2.contourArea(contours[i])
		
		# Print the area or centroid in the image (for debugging purposes)
		font = cv2.FONT_HERSHEY_SIMPLEX
		imgsal = cv2.putText(cv_image,str(area),(cx_avg,cy_avg), font, 2,(0,0,0),2,cv2.LINE_AA)	
		cv2.imshow('image 4',imgsal)
		
		objInfo = img_obj_info()			#Custom message type instance
		# Custom message created consists of cx, cy and area
		objInfo.cx = cx_avg
		objInfo.cy = cy_avg
		objInfo.area = area
		self.objInfo.publish(objInfo)		# Publish data to ObjInfo topic so it can be retreived later

		return contours, mask_eroded_dilated
		
	def showImage(self,cv_image, mask_erode_image, mask_image):
		# Thresholded and original images
		res = cv2.bitwise_and(cv_image, cv_image, mask = mask_image)	    
		
		# Show images on cv window 
		#  cv2.imshow('OpenCV_Original', cv_image)
		#  cv2.imshow('OpenCV_Mask_Erode', mask_erode_image)
		#  cv2.imshow('OpenCV_Mask_Dilate', mask_image)
		#  cv2.imshow("OpenCV_Image", res)
		#  cv2.waitKey(3)
		self.image_pub_res.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
		
	def objectData(self,data):
		centerx = data.cx
		centery = data.cy
		area = data.area
		
		return area, centerx, centery
		
		
#**************************************DRONE FUNCTIONS**************************************************************

class drone_cmds:
	
	def __init__(self):
		#===============================================
		#	 CHANGE TOPICS NECESARY FOR THE DRONE 
		#===============================================
		
		self.topictakeoff = rospy.Publisher("/bebop/takeoff",Empty,queue_size = 1)
		self.topicland = rospy.Publisher("/bebop/land",Empty,queue_size = 1)
		self.cmd_vel =  rospy.Publisher("bebop/cmd_vel",Twist,queue_size = 1)
		self.topicflattrim =  rospy.Publisher("/bebop/flattrim",Empty,queue_size = 1)
		
		# Subscribe to ObjInfo topic so the centroid and area can be retreived and used to follow the object
		self.ObjInfo = rospy.Subscriber("/ObjInfo",img_obj_info,self.callback)
		
	def callback(self,data):
		#Data is the custom message created and retrieved from /ObjInfo topic
		centerx = data.cx
		centery = data.cy
		area = data.area		
		if (area == 0):
			self.stop()
		
		#Pass data message to move() method to carry the following object task
		self.move(area)
		#print ("centroid: {}, {} ~~ area: {}".format(cx,cy,area))
		
	def changeTwist(self, x, y, z, turn):
		#Necessary commands for movement in yaw, pitch, roll axes
		msg_vel = Twist()
		msg_vel.angular.x = 0
		msg_vel.angular.y = 0;
		msg_vel.angular.z = turn;
		msg_vel.linear.x = x;
		msg_vel.linear.y = y;
		msg_vel.linear.z = z;
		
		return(msg_vel)
		
	def adjust(self):
		#Horizontal adjust (Calibration)
		empty_msg = Empty()		#Empty message instance
		self.topicflattrim.publish(empty_msg)
		print("Calibrating...\n")
		time.sleep(2)
		
	def takeoff(self):
		#Takeoff and hovering
		#Publish into /bebop/takeoff topic
		empty_msg = Empty()
		msg_vel = Twist()
		self.topictakeoff.publish(empty_msg)
		print("Starting...\n")
		time.sleep(.5)
		print("Changing velocity for hovering...\n")
		msg_vel = self.changeTwist(0,0,0,0)
		self.cmd_vel.publish(msg_vel)
		
	def land(self):
		#Landing
		#Publish into topic /bebop/land
		empty_msg = Empty()
		self.topicland.publish(empty_msg)
	
	def forwardx(self):
		msg_vel = Twist()
		#Velocity info: velocity goes from 0 to 1 to go forward
		#Here a velocity of 0.1 is being published to the x axis
		msg_vel = self.changeTwist(.1,0,0,0)
		self.cmd_vel.publish(msg_vel)
		print ('ForwardX')

	def backwardx(self):
		msg_vel = Twist()  
		#Velocity info: velocity goes from 0 to -1 to go backward
		msg_vel = self.changeTwist(-.1,0,0,0)
		self.cmd_vel.publish(msg_vel)
		print ('BackX')

	def left(self):
		msg_vel = Twist() 
		#Velocity info
		msg_vel = self.changeTwist(0,.1,0,0)
		self.cmd_vel.publish(msg_vel)
		print ('Left')

	def right(self):
		msg_vel = Twist()
		#Velocity info
		msg_vel = self.changeTwist(0,-.1,0,0)
		self.cmd_vel.publish(msg_vel)
		print ('Right')
		
	def up(self):
		msg_vel = Twist()  
		#Velocity info
		msg_vel = self.changeTwist(0,0,.1,0)
		self.cmd_vel.publish(msg_vel)
		print ('Up')
		
	def down(self):
		msg_vel = Twist()
		#Velocity info
		msg_vel = self.changeTwist(0,0,-.1,0)
		self.cmd_vel.publish(msg_vel)
		print ('Down')
		
	def stop(self):
		msg_vel = Twist() 
		#Velocity info
		msg_vel = self.changeTwist(0,0,0,0)
		self.cmd_vel.publish(msg_vel)
		print ('Stop')
		
	def move(self,area):
		#Target tracking
		#Area values may change depending on the application or the object
		if 0 < area < 2500:
			self.forwardx()
			temp1 = True		#Temporal variable
	
		elif area > 3000:
			self.backwardx()
			temp1 = True
			
		elif area < 3000 and area > 2500:
			if temp1 == True:
				self.stop()
				temp1 = False
			#if y < 200:			#Check the Y position of the centroid
				#self.up()
				#temp2 = True		#Temporal variable
				
			#elif y > 300:
				#self.down()
				#temp2 = True
				
			#elif y < 300 and y > 200:
				#if temp2 == True:
					#self.stop()
					#temp2 = False
				#if x < 400:		#Check X position of the centroid
					#self.left()
					
				#elif x > 440:
					#self.right()
					
				#elif x < 440 and x > 400:
					#self.stop()					
				
def main(args):
	#Declare and call classes
	drone = drone_cmds()
	
	print("Starting image..")
	rospy.sleep(.5)
	ic = image_convert()
	
	rospy.init_node('Drone_Project', anonymous=True)		#Initialize node
	while not rospy.is_shutdown():
		#To handle errors and prevent program's sudden shutdown use try-except
		try:
			print("Starting control..")
			drone.adjust()		#Call to adjust() method
			rospy.sleep(.5)
			print("Press 'T' for takeoff and 'Spacebar' for landing\n")
			key = getkeys()
			#key = raw_input("Presione 'T' para despegar\n>> ")
			if key == 't':
				drone.takeoff()
			else:
				drone.land()
			if key == 'space':
				drone.land()
				break
			else:
				drone.land()
				
		except KeyboardInterrupt:
			drone.land()
			print("Saliendo..")
			break
			
		rospy.sleep(1)
		rospy.spin()		#Prevents node from exiting until it is shutdown
	
	cv2.destroyAllWindows()


if __name__ == '__main__':
    print("Staring..")
    rospy.sleep(1)
    main(sys.argv)
      
