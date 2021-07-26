import time

import cv2
import face_recognition as fr
import numpy as np
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

video_capture = cv2.VideoCapture(-1)

me_image = fr.load_image_file("me.jpg")
me_face_encoding = fr.face_encodings(me_image)[0]

known_face_encondings = [me_face_encoding]
known_face_names = ["Me"]

while True: 
    ret, frame = video_capture.read()
    frame = cv2.flip(frame, -1) #camera flip
    small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
    rgb_frame = small_frame[:, :, ::-1]

    face_locations = fr.face_locations(rgb_frame)
    face_encodings = fr.face_encodings(rgb_frame, face_locations)

    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):

        matches = fr.compare_faces(known_face_encondings, face_encoding)

        name = "Unknown"

        face_distances = fr.face_distance(known_face_encondings, face_encoding)

        best_match_index = np.argmin(face_distances)
        if matches[best_match_index]:
            name = known_face_names[best_match_index]
                   
        cv2.rectangle(frame, (left * 2, top * 2), (right * 2, bottom * 2), (0, 0, 255), 2)

        #cv2.rectangle(frame, (left, bottom * 4 -35), (right, bottom), (0, 0, 255), cv2.FILLED)
         
        font = cv2.FONT_HERSHEY_SIMPLEX
        GPIO.output(18, True) 
        cv2.putText(frame, name, (left * 2 + 6, bottom * 2 - 6), font, 1.0, (255, 255, 255), 1)  
        time.sleep(1)
        GPIO.output(18, False)

    cv2.imshow('Webcam_facerecognition', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        GPIO.cleanup(18)
        break

video_capture.release()
cv2.destroyAllWindows()
