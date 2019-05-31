#!/usr/bin/env python
import cv2
import sys
import numpy as np



def nothing(x):
	pass

cv2.namedWindow("Threshold")
cv2.createTrackbar("LH", "Threshold", 0, 255, nothing)
cv2.createTrackbar("LS", "Threshold", 0, 255, nothing)
cv2.createTrackbar("LV", "Threshold", 0, 255, nothing)
cv2.createTrackbar("UH", "Threshold", 255, 255, nothing)
cv2.createTrackbar("US", "Threshold", 255, 255, nothing)
cv2.createTrackbar("UV", "Threshold", 255, 255, nothing)

def main(args):
	while True:
		imagen = 'turtle1.png'
		frame = cv2.imread(imagen)
		hsv =  cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		
		lh = cv2.getTrackbarPos("LH", "Threshold")
		ls = cv2.getTrackbarPos("LS", "Threshold")
		lv = cv2.getTrackbarPos("LV", "Threshold")
		uh = cv2.getTrackbarPos("UH", "Threshold")
		us = cv2.getTrackbarPos("US", "Threshold")
		uv = cv2.getTrackbarPos("UV", "Threshold")
		
		l_r = np.array([lh,ls,lv])
		u_r = np.array([uh,us,uv])
		
		mask = cv2.inRange(hsv,l_r,u_r)
		res = cv2.bitwise_and(frame, frame, mask=mask)
		cv2.imshow("frame",frame)
		cv2.imshow("IMG",res)
		
		
		key = cv2.waitKey(1)
		if key == 27:
			break
		print("lh:{}\nls:{}\nlv:{}\nuh:{}\nus:{}\nuv:{}\n".format(lh,ls,lv,uh,uv,us))
				
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
