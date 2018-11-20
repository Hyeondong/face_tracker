#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them,
then centers the webcam via two servos so the face is at the center of the screen
Based on facedetect.py in the OpenCV samples directory
"""
import sys
from optparse import OptionParser
from imutils.video import VideoStream
import imutils
import RPi.GPIO as GPIO
import datetime
import cv2
import time
import os

face_id = "photo"
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT)
GPIO.output(20, False)


min_size = (20, 20)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
detector = cv2.CascadeClassifier("/home/pi/Downloads/pi-face-recognition/haarcascade_frontalface_default.xml")
smilePath = "/home/pi/Downloads/pi-face-recognition/haarcascade_smile.xml"
smileCascade = cv2.CascadeClassifier(smilePath)
terminate = False

max_pwm = 249
min_pwm = 1
midScreenWindow = 40  # acceptable 'error' for the center of the screen.
panStepSize = 2 # degree of change for each pan update
tiltStepSize = -2 # degree of change for each tilt update
servoPanPosition = 125 # initial pan position
servoTiltPosition = 160 # initial tilt position
panGpioPin = 2  # servoblaster pin 2 : gpio pin 18
tiltGpioPin = 5  # servoblaster pin 5 : gpio pin 23

def detect_and_draw(frame):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    midFace = None

    width = gray.shape[1]
    height = gray.shape[0]
    sum = 0
    cnt = 0

    for i in range(50, height-50, 20):
		for j in range(50, width -50, 20):
			sum += gray[i,j]
			cnt += 1

	avg = sum/cnt
	if avg < 30:
		print("dark")
		GPIO.output(20, True)
	else:
		GPIO.output(20, False)

    faces = detector.detectMultiScale(
            gray, scaleFactor=1.2,
            minNeighbors=2,
            minSize=(20, 20)
    )

    for (x, y, w, h) in faces:
        # the input to cv.HaarDetectObjects was resized, so scale the
        # bounding box of each face and convert it to two CvPoints
        pt1 = (int(x * image_scale), int(y * image_scale))
        pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))

        # get the xy corner co-ords, calc the midFace location
        x1 = pt1[0]
        x2 = pt2[0]
        y1 = pt1[1]
        y2 = pt2[1]
        midFaceX = x1+((x2-x1)/2)
        midFaceY = y1+((y2-y1)/2)
        midFace = (midFaceX, midFaceY)

        #cv.Rectangle(frame, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)
        save_frame = frame
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        roi_gray = gray[y:y+h, x:x+w]
		roi_color = frame[y:y+h, x:x+w]

        smile = smileCascade.detectMultiScale(
			roi_gray,
			scaleFactor= 1.6,
			minNeighbors=10,
#			minSize=(10, 10),
#			maxSize=(30, 30),
			flags=cv2.CASCADE_SCALE_IMAGE
		)

        for (x2, y2, w2, h2) in smile:
			now = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
			cv2.rectangle(roi_color, (x2, y2), (x2+w2, y2+h2), (0, 255, 0), 2)
			cv2.putText(frame,'Smile',(x,y-7), 3, 1.2, (0, 255, 0), 2, cv2.LINE_AA)
			cv2.imwrite("/media/pi/ESD-USB1/"+"photo/"+now+".jpg", save_frame)
			cv2.imshow("Frame", frame)
			key2 = cv2.waitKey(1) & 0xFF
			if key2 == ord("q"):
                terminate = True
				break
			time.sleep(2.0)



    cv2.imshow("Frame", frame)
    return midFace

def move(servo, angle):
    '''Moves the specified servo to the supplied angle.

    Arguments:
        servo
          the servo number to command, an integer from 0-7
        angle
          the desired pulse width for servoblaster, an integer from 0 to 249

    (e.g.) >>> servo.move(2, 90)
           ... # "move servo #2 to 90 degrees"'''

    if (min_pwm <= angle <= max_pwm):
        command = 'echo %s=%s > /dev/servoblaster' % (str(servo), str(angle))
        os.system(command)
        #print command
    else:
        print("Servo angle must be an integer between 0 and 249.\n")

if __name__ == '__main__':

    capture = VideoStream(src=0).start()
    time.sleep(1.0)

    if capture:
        move(panGpioPin, servoPanPosition)
        move(tiltGpioPin, servoTiltPosition)

        while True:
            frame = capture.read()
            frame = imutils.resize(frame, width=500)

            midScreenX = (frame.shape[1]/2)
            midScreenY = (frame.shape[0]/2)

            midFace = detect_and_draw(frame)

            if midFace is not None:
                midFaceX = midFace[0]
                midFaceY = midFace[1]

                #얼굴이 화면 왼쪽에 위치할 경우.
                if(midFaceX < (midScreenX - midScreenWindow)):
                    servoPanPosition += panStepSize
                    print(str(midFaceX) + " > " + str(midScreenX) + " : Pan Right : " + str(servoPanPosition))
                #얼굴이 화면 오른쪽에 위치할 경우.
                elif(midFaceX > (midScreenX + midScreenWindow)):
                    servoPanPosition -= panStepSize
                    print(str(midFaceX) + " < " + str(midScreenX) + " : Pan Left : " + str(servoPanPosition))
                else:
                    print(str(midFaceX) + " ~ " + str(midScreenX) + " : " + str(servoPanPosition))

                servoPanPosition = min(servoPanPosition, max_pwm)
                servoPanPosition = max(servoPanPosition, min_pwm)
                move(panGpioPin, servoPanPosition)

                if(midFaceY > (midScreenY + midScreenWindow)):
                    if(servoTiltPosition >=1):
                        #Update the tilt position variable to lower the tilt servo.
                        servoTiltPosition -= tiltStepSize
                        print(str(midFaceY) + " > " + str(midScreenY) + " : Tilt Down : " + str(servoTiltPosition))
                #Find out if the Y component of the face is above the middle of the screen.
                #elif(midFaceY > (midScreenY + midScreenWindow)):
                   #if(servoTiltPosition >= 1):
                elif(midFaceY < (midScreenY - midScreenWindow)):
                    if(servoTiltPosition <= max_pwm):
                        #Update the tilt position variable to raise the tilt servo.
                        servoTiltPosition += tiltStepSize
                        print(str(midFaceY) + " < " + str(midScreenY) + " : Tilt Up : " + str(servoTiltPosition))
                else:
                    print(str(midFaceY) + " ~ " + str(midScreenY) + " : " + str(servoTiltPosition))

                servoTiltPosition = min(servoTiltPosition, max_pwm)
                servoTiltPosition = max(servoTiltPosition, min_pwm)
                move(tiltGpioPin, servoTiltPosition)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                GPIO.output(20, False)
                break

    cv2.destroyAllWindows()
    capture.stop()
