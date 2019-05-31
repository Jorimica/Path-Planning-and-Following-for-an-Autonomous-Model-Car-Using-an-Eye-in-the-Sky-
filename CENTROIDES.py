#include <opencv2\opencv.hpp>
import cv2
import numpy as np
#using namespace cv;

img = cv2.imread('rojos.png',0)
ret,thresh = cv2.threshold(img,127,255,0)

cv2.imshow('image 1',thresh)
Key=cv2.waitKey(2000)
img,contours,hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
cv2.imshow('image 2',img)
Key=cv2.waitKey(2000)




N=len(contours)
print N
cx1=0
cy1=0
cx2=0
cy2=0
for i in contours[0:2]:
	a = contours.index(i)
	print a
	cnt = contours[a]
	print cnt
	M = cv2.moments(cnt)
	print M
	cx = int(M['m10']/M['m00'])
	cy = int(M['m01']/M['m00'])
	cx1=cx+cx1
	cy1=cy+cy1
	print cx
	print cy
cx2=cx1/2
cy2=cy1/2

	
font = cv2.FONT_HERSHEY_SIMPLEX
imgsal = cv2.putText(img,str(a),(cx2,cy2), font, 2,(255,255,255),2,cv2.LINE_AA)	
cv2.imshow('image 4',imgsal)
Key=cv2.waitKey(10000)
	
	
if Key==27:
	cv2.destroyAllWindows()


