import time

import cv2
import face_recognition as fr
import numpy as np
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import adafruit_dht
import board
import drivers

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connected OK")
    else:
        print("Bad connection Returned code=", rc)


def on_disconnect(client, userdata, flags, rc=0):
    print(str(rc))


def on_subscribe(client, userdata, mid, granted_qos):
    print("subscribed: " + str(mid) + " " + str(granted_qos))


def on_message(client, userdata, msg):

    if str(msg.payload.decode("utf-8")) == "on":
        print(str(msg.payload.decode("utf-8")))
        face_recog_receive()
    elif str(msg.payload.decode("utf-8")) == "off":
        raise KeyboardInterrupt
        
flag = 0        
def face_recog_receive():
    LED = 23
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED, GPIO.OUT)

    sensor = adafruit_dht.DHT11(board.D24)

    # HC-SR04의 트리거 핀을 GPIO 28번, 에코핀을 GPIO 29번에 연결한다.
    GPIO_TRIGGER = 20 # 28 
    GPIO_ECHO = 21    # 29
 
    # 초음파를 내보낼 트리거 핀은 출력 모드로, 반사파를 수신할 에코 피은 입력 모드로 설정한다.
    GPIO.setup(GPIO_TRIGGER,GPIO.OUT) 
    GPIO.setup(GPIO_ECHO,GPIO.IN)
    
    display = drivers.Lcd()
        
    video_capture = cv2.VideoCapture(0)

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

            font = cv2.FONT_HERSHEY_SIMPLEX
            
            GPIO.output(LED, True) 
            
            c = sensor.temperature
            h = sensor.humidity
            if h is not None and c is not None :
                print("Temperature = {0:0.1f}*C Humidity = {1:0.1f}%".format(c, h))
            
            else :
                print('Read error')
            
            stop = 0
            start = 0
            # 먼저 트리거 핀을 OFF 상태로 유지한다
            GPIO.output(GPIO_TRIGGER, False)
            time.sleep(2)
    
            # 10us 펄스를 내보낸다. 
            # 파이썬에서 이 펄스는 실제 100us 근처가 될 것이다.
            # 하지만 HC-SR04 센서는 이 오차를 받아준다.
            GPIO.output(GPIO_TRIGGER, True)
            time.sleep(0.00001)
            GPIO.output(GPIO_TRIGGER, False)
    
            # 에코 핀이 ON되는 시점을 시작 시간으로 잡는다.
            while GPIO.input(GPIO_ECHO)==0:
                start = time.time()
    
            # 에코 핀이 다시 OFF되는 시점을 반사파 수신 시간으로 잡는다.
            while GPIO.input(GPIO_ECHO)==1:
                stop = time.time()
    
            # Calculate pulse length
            elapsed = stop-start
    
            # 초음파는 반사파이기 때문에 실제 이동 거리는 2배이다. 따라서 2로 나눈다.
            # 음속은 편의상 340m/s로 계산한다. 현재 온도를 반영해서 보정할 수 있다.
            if (stop and start):
                distance = elapsed * 17000.0
                print("Distance : %.1f cm" % distance)
            
            name += "distance = {0:0.1f}cm, Temperature = {1:0.1f}*C, Humidity = {2:0.1f}%".format(distance, c, h)
            
            lcd_distance = "distance = {0:0.1f}cm".format(distance)
            lcd_temperture = "Temperature = {0:0.1f}*C".format(c)
            lcd_humidity = "Humidity = {0:0.1f}%".format(h)
            display.lcd_display_string(lcd_distance, 1)
            display.lcd_display_string(lcd_temperture, 2)
            
            cv2.putText(frame, name, (left * 2 + 6, bottom * 2 - 6), font, 1.0, (255, 255, 255), 1)  
            
            time.sleep(2)
            GPIO.output(LED, False)

        cv2.imshow('Webcam_facerecognition', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            GPIO.cleanup(LED)
            GPIO.cleanup(GPIO_TRIGGER)
            GPIO.cleanup(GPIO_ECHO)
            
            break
            
    video_capture.release()
    cv2.destroyAllWindows()
    raise KeyboardInterrupt

# 새로운 클라이언트 생성
client = mqtt.Client()

# 콜백 함수 설정 on_connect(브로커에 접속), on_disconnect(브로커에 접속중료), on_subscribe(topic 구독),
# on_message(발행된 메세지가 들어왔을 때)
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_subscribe = on_subscribe
client.on_message = on_message

# 로컬 아닌, 원격 mqtt broker에 연결
# address: localhost
# port: 1883 에 연결
client.connect('localhost', 1883)

# topic 구독
client.subscribe('test', 1)
client.loop_forever()

