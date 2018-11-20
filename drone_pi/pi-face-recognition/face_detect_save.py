from imutils.video import VideoStream
import imutils
import time
import cv2

time.sleep(10.0)
vs = VideoStream(src=0).start()
time.sleep(2.0)
face_detector = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')


face_id = "photo"
count = 0
print("streaming...")
while True:
    image_frame = vs.read()
    image_frame = imutils.resize(image_frame, width=500)

    gray = cv2.cvtColor(image_frame, cv2.COLOR_BGR2GRAY)

    faces = face_detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

    for (x,y,w,h) in faces:
      #  cv2.rectangle(image_frame, (x,y), (x+w, y+h), (255, 0, 0), 2)
        
        cv2.imwrite("/media/pi/ESD-USB/"+face_id+"/"+str(count)+".jpg", image_frame)
       # cv2.imshow('frame', image_frame)
        count+=1
        print("detected..")

    if cv2.waitKey(100) & 0xFF == ord('q'):
        break

    elif count>7:
        break

cv2.destroyAllWindows()
vs.stop()
