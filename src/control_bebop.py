#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('drone')
import sys
import rospy
import numpy as np
import cv2
import time
#from std_srvs.srv import Empty
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from cv_bridge import CvBridge, CvBridgeError

#*********************IDENTIFICADOR DE OBJETOS POR COLOR**********************************************
class image_convert:

	def __init__(self):
		#===============================================
		#	CAMBIAR TOPICOS NECESARIOS PARA EL DRON
		#===============================================
		self.area = None
		self.centerx = None
		self.centery = None
		self.image_pub = rospy.Publisher("/image_converter/output_video",Image,queue_size = 10)
		self.image_pub_res = rospy.Publisher("/object_detection/img_result", Image, queue_size=10)
		
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/bebop/image_raw",Image,self.callback)

	def callback(self,data):
		try:
			
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		#(rows,cols,channels) = cv_image.shape
		#if cols > 60 and rows > 60 :
		#cv2.circle(cv_image, (50,50), 10, 255)
		
 #--------------AGREGADO---------------------------------------------------------------------     
		contours, mask_image = self.HSVObjectDetection(cv_image)
		
		for cnt in contours:
			# Encontrar bounding box del objeto detectado
			#  xp, yp son las coordenadas de la esquina superior izq. del bounding box
			#  w, h son el ancho (width) y altura (height) del rectangulo
			xp,yp,w,h = cv2.boundingRect(cnt) 
			
			cv2.rectangle(cv_image,(xp,yp),(xp+w,yp+h),[0,255,255], 2)
			centerx, centery = xp+w/2, yp+h/2

#--------------------------------------------------------------------------------------------		
		#cv2.imshow("Original Image", cv_image)
		#cv2.waitKey(3)
		
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)
			
			
#-------------AGREGADO-----------------------------------------------------------------------			
		
	def HSVObjectDetection(self,cv_image, toPrint = True):
		hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		
		# Definir el rango de color en HSV
		lower_red = np.array([000,180,129])
		upper_red = np.array([012,255,255])
		
		# Umbralizacion de color de la imagen HSV
		mask = cv2.inRange(hsv_image, lower_red, upper_red)   ##
		mask_eroded = cv2.erode(mask, None, iterations = 2)  ##
		mask_eroded_dilated = cv2.dilate(mask_eroded, None, iterations = 10)  ##
    
		#if toPrint:
			#print ('hsv'), hsv_image[240][320] # the center point hsv
			
		self.showImage(cv_image, mask_eroded, mask_eroded_dilated)
		_,contours, _ = cv2.findContours(mask_eroded_dilated,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		index=0
		cx1=0
		cy1=0
		cx2=0
		cy2=0
		N=len(contours)
		print (N)
		for i in contours[0:1]:
			index = contours.index(i)
			#print a
			cnt = contours[index]
			#print cnt
			M = cv2.moments(cnt)
			#print M
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cx1=cx+cx1
			cy1=cy+cy1
			#print cx
			#print cy
		cx2=cx1/1
		cy2=cy1/1
		font = cv2.FONT_HERSHEY_SIMPLEX
		imgsal = cv2.putText(cv_image,str(cx2),(cx2,cy2), font, 2,(255,255,255),2,cv2.LINE_AA)	
		cv2.imshow('image 4',imgsal)
		Key=cv2.waitKey(3)
		
		
		for i in range(len(contours)):
			self.area = cv2.contourArea(contours[i])
			#print("area: {}".format(self.area))
		return contours, mask_eroded_dilated
		
	def showImage(self,cv_image, mask_erode_image, mask_image):
		# Imagen binarizada e imagen original
		res = cv2.bitwise_and(cv_image, cv_image, mask = mask_image)	    
		# Draw a cross at the center of the image
		#cv2.line(cv_image, (320, 235), (320, 245), (255,0,0))
		#cv2.line(cv_image, (325, 240), (315, 240), (255,0,0))
		
		# Mostrar imagenes en cv window 
		#  cv2.imshow('OpenCV_Original', cv_image)
		#  cv2.imshow('OpenCV_Mask_Erode', mask_erode_image)
		#  cv2.imshow('OpenCV_Mask_Dilate', mask_image)
		cv2.imshow("OpenCV_Image", res)
		cv2.waitKey(3)
		self.image_pub_res.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
		
	def objectArea(self):
		return self.area, self.centerx, self.centery
#--------------------------------------------------------------------------------------------
		
		
#**************************************FUNCIONES DEL DRON**************************************************************

class drone_cmds:
	def __init__(self):
		#===============================================
		#	CAMBIAR TOPICOS NECESARIOS PARA EL DRON
		#===============================================
		
		self.topictakeoff = rospy.Publisher("/bebop/takeoff",Empty,queue_size = 1)
		self.topicland = rospy.Publisher("/bebop/land",Empty,queue_size = 1)
		self.cmd_vel =  rospy.Publisher("/cmd_vel",Twist,queue_size = 1)
		#rospy.wait_for_service('/bebop/flattrim')
		#self.serviceflattrim =  rospy.ServiceProxy("/bebop/flattrim",Empty)
		self.topicflattrim =  rospy.Publisher("/bebop/flattrim",Empty,queue_size = 1)
		
	def changeTwist(self, x, y, z, turn):
		#Comandos necesarios para el movimiento en los ejes yaw, pitch, roll
		msg_vel = Twist()
		msg_vel.angular.x = 0
		msg_vel.angular.y = 0;
		msg_vel.angular.z = turn;
		msg_vel.linear.x = x;
		msg_vel.linear.y = y;
		msg_vel.linear.z = z;
		
		return(msg_vel)
		
	def adjust(self):
		#Ajuste horizontal (Calibracion)
		#Llamada al servicio que permite el ajuste(Sincrono)
		#srvflattrim = std_srvs.Empty
		#self.serviceflattrim(srvflattrim)
		empty_msg = Empty()
		self.topicflattrim.publish(empty_msg)
		print("Calibrando\n")
		time.sleep(2)
		
	def takeoff(self):
		#Desepegue y hovering
		#Se publica en el topico /bebop/takeoff (Asincrono)
		empty_msg = Empty()
		msg_vel = Twist()
		self.topictakeoff.publish(empty_msg)
		print("Comenzando...\n")
		time.sleep(1)
		print("Cambiando velocidad para hovering...\n")
		msg_vel = self.changeTwist(0,0,0,0)
		self.cmd_vel.publish(msg_vel)
		
	def land(self):
		#Aterrizaje
		#Se publica en el topico /bebop/land
		empty_msg = Empty()
		self.topicland.publish(empty_msg)
	
	def forwardx(self):
		msg_vel = Twist()  #info velocidad
		msg_vel = self.changeTwist(1,0,0,0);
		self.cmd_vel.publish(msg_vel)

	def backwardx(self):
		msg_vel = Twist()  #info velocidad
		msg_vel = self.changeTwist(-1,0,0,0);
		self.cmd_vel.publish(msg_vel)

	def forwardy(self):
		msg_vel = Twist()  #info velocidad
		msg_vel = self.changeTwist(0,1,0,0);
		self.cmd_vel.publish(msg_vel)

	def backwardy(self):
		msg_vel = Twist()  #info velocidad
		msg_vel = self.changeTwist(0,-1,0,0);
		self.cmd_vel.publish(msg_vel)
		
	def up(self):
		msg_vel = Twist()  #info velocidad
		msg_vel = self.changeTwist(0,0,1,0);
		self.cmd_vel.publish(msg_vel)
		
	def down(self):
		msg_vel = Twist()  #info velocidad
		msg_vel = self.changeTwist(0,0,-1,0);
		self.cmd_vel.publish(msg_vel)
		
	def stop(self):
		msg_vel = Twist()  #info velocidad
		msg_vel = self.changeTwist(0,0,0,0);
		self.cmd_vel.publish(msg_vel)
		
	def move(self,area, x, y):
		print ('ya entre')
		#Seguimiento del objeto
		#Cambiar valores de area si es necesario
		if area == 0:
			self.stop()
			print ('stop1')
		if area < 2500:
			print ('ForX')
			self.forwardx()
			temp1 = True
			
		if area > 3000:
			print ('BackX')
			self.backwardx()
			temp1 = True
			
		if area < 3000 and area > 2500:
			if temp1 == True:
				print ('stop2')
				self.stop()
				temp1 = False
			if y < 200:
				self.up()
				temp2 = True
				print ('UP')
			if y > 300:
				self.down()
				print ('down')
				temp2 = True
			if y < 400 and y > 370:
				if temp2 == True:
					self.stop()
					print ('stop3')
					temp2 = False
				if x > 400:
					self.backwardy()
					print ('backy')
				if x < 440:
					self.forwardy()
					print ('forwardy')
				if x < 440 and x > 400:
					self.stop()	
					print ('stop4')					
				
def main(args):
  drone = drone_cmds()
  
  rospy.init_node('Proyecto', anonymous=True)
 
  
  
  try:
	  print("Iniciando control")
	  drone.adjust()
	  print("Iniciando imagen")
	  	  
	  
	  key = raw_input("Presione 'T' para despegar\n>> ")
 
	  if key.upper() == 'T':
		  ic = image_convert()
		  area, x, y = ic.objectArea()
		  drone.takeoff()
		  drone.move(area, x, y)
		  	
	  else:
		  drone.land()
		  
		
  except KeyboardInterrupt:
	  print("Saliendo")
  rospy.spin()
  cv2.destroyAllWindows()


if __name__ == '__main__':
    print("Iniciando...")
    main(sys.argv)
      
