#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Int8
import random, time


class numero:
	def __init__(self):
		self.num_pub = rospy.Publisher('chatter', String, queue_size=10)
		self.num_sub = rospy.Subscriber("chatter", String,self.suscribir)
		self.letra = String()
		
	def publicar(self):
		#num = random.randint(0,50)
		num = 'a'
		time.sleep(1)
		self.num_pub.publish(num)
		
	def suscribir(self,msg):
		self.letra = msg.data
		rospy.loginfo(self.letra)
		#num = Int8.data
		#if 0 < num <= 10:
			#print("{} -- Menos de 10".format(data))
		#elif 10 < num <= 20:
			#print("{} -- Entre 10 y 20".format(data))
		#elif 20 < num <= 30:
			#print("{} -- Entre 20 y 30".format(data))
		#else:
			#print("{} -- Mayor a 30".format(data))
		if msg == 'a':
			print(self.letra)
			
def main(args):
	x = 0
	n = numero()
	rospy.init_node('Numero', anonymous=True)
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		n.publicar()
		rospy.sleep(1)
	rospy.spin()

if __name__ == '__main__':
    print("Iniciando...")
    main(sys.argv)
