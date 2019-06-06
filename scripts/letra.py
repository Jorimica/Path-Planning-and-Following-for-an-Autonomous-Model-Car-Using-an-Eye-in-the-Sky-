#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Int8
import random, time

class letras:
	def __init__(self):
		self.num_sub = rospy.Subscriber("chatter", String,self.suscribir)
		self.letra = String()
		
	def suscribir(self,msg):
		msn=String()
		self.letra = msg.data
		#print(self.letra)
		#num = Int8.data
		#if 0 < num <= 10:
			#print("{} -- Menos de 10".format(data))
		#elif 10 < num <= 20:
			#print("{} -- Entre 10 y 20".format(data))
		#elif 20 < num <= 30:
			#print("{} -- Entre 20 y 30".format(data))
		#else:
			#print("{} -- Mayor a 30".format(data))
		if self.letra == "z":
			print(self.letra)
		else: 
			print("Uriel")
	
		return self.letra
			
if __name__ == '__main__':
	l = letras()
	rospy.init_node('susc', anonymous=True)
	rate = rospy.Rate(5)
	rospy.spin()
	
