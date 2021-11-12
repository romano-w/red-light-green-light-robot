import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

cap = cv2.VideoCapture(0)
face_cas =cv2.CascadeClassifier('Haar_Cascades/frontal_face.xml')

while True:
	_ , frame = cap.read()
	frame = cv2.resize(frame, (640, 360))

	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	face = face_cas.detectMultiScale(gray)

	for (x,y,w,h) in face:
		cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0), 5)


	result = cv2.flip(frame,1)
	
	cv2.imshow('Result', result)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
