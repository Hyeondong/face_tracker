# USAGE

from imutils.video import VideoStream
import face_recognition
import imutils
import time
import cv2
import RPi.GPIO as GPIO

face_id = "photo"
count=0

print("[INFO] loading encodings + face detector...")
#data = pickle.loads(open(args["encodings"], "rb").read())
detector = cv2.CascadeClassifier("/home/pi/Downloads/pi-face-recognition/haarcascade_frontalface_default.xml")
#facePath = "lbpcascade_frontalface.xml"
#detector = cv2.CascadeClassifier(facePath)

smilePath = "haarcascade_smile.xml"
smileCascade = cv2.CascadeClassifier(smilePath)


print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)

while True:
	frame = vs.read()
	frame = imutils.resize(frame, width=500)
	
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	width = gray.shape[1]
	height = gray.shape[0]
	sum = 0
	count = 0


	for i in range(50, height-50, 20):
		for j in range(50, width-50, 20):
			sum += gray(i,j)
			count += 1
	avg = sum/count
	if avg < 50:
		GPIO.output(23, True)



	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	if key == ord("q"):
		break
	if count>5:
		break

print("finish")

cv2.destroyAllWindows()
vs.stop()

