import RPi.GPIO as GPIO
import time

def led(pin, t):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.OUT)

    GPIO.output(pin, True)
    time.sleep(t) 

    GPIO.cleanup(pin)

led(23, 5) # 18번 핀(GPIO 24)에 끼운 LED를 5초동안 점등
