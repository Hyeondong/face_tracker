# USAGE

from imutils.video import VideoStream
import face_recognition
import imutils
import time
import cv2

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


while True:
	frame = vs.read()
	frame = imutils.resize(frame, width=500)
	
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	#rects = detector.detectMultiScale(gray, scaleFactor=1.1, 
	#	minNeighbors=5, minSize=(30, 30),
	#	flags=cv2.CASCADE_SCALE_IMAGE)

	#for (x,y,w,h) in rects:
		#  cv2.rectangle(image_frame, (x,y), (x+w, y+h), (255, 0, 0), 2)
	#	cv2.imwrite("/media/pi/ESD-USB1/"+face_id+"/"+str(count)+".jpg", frame)
#		 cv2.imshow('frame', image_frame)
	#	count+=1
	#	print("detected..")


	faces = detector.detectMultiScale(
		gray,
		scaleFactor= 1.3,
		minNeighbors=5
	)



	for (x, y, w, h) in faces:
#		cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
		roi_gray = gray[y:y+h, x:x+w]
		roi_color = frame[y:y+h, x:x+w]

		smile = smileCascade.detectMultiScale(
			roi_gray,
			scaleFactor= 1.16,
			minNeighbors=35,
#			minSize=(10, 10),
#			maxSize=(30, 30),
			flags=cv2.CASCADE_SCALE_IMAGE
		)

		for (x2, y2, w2, h2) in smile:
			print("write")
			cv2.imwrite("/media/pi/ESD-USB1/"+face_id+"/"+str(count)+".jpg", frame)
			count+=1
			#cv2.rectangle(roi_color, (x2, y2), (x2+w2, y2+h2), (0, 255, 0), 2)
			#cv2.putText(frame,'Smile',(x,y-7), 3, 1.2, (0, 255, 0), 2, cv2.LINE_AA)

	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	if key == ord("q"):
		break
	if count>5:
		break

print("finish")

cv2.destroyAllWindows()
vs.stop()

