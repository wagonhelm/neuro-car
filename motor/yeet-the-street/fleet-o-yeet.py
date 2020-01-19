import sys
import socketio
import Jetson.GPIO as GPIO
import time

pulled_over = False

stop_pin = 5 #29
forward_pin = 6 #31
pull_over_pin = 13 #33
pump_brake_pin = 19 #35

sio = socketio.Client()

GPIO.setmode(GPIO.BCM)
GPIO.setup(stop_pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(forward_pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(pull_over_pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(pump_brake_pin, GPIO.OUT, initial=GPIO.LOW)

@sio.event
def connect():
	print("Connected!")


@sio.event
def disconect():
	print("Got disconnected!")

	# TODO: stop motors
	GPIO.output(stop_pin, GPIO.LOW)
	GPIO.output(forward_pin, GPIO.LOW)
	GPIO.output(pull_over_pin, GPIO.LOW)
	GPIO.output(pump_brake_pin, GPIO.LOW)
	sys.exit(0)

@sio.event
def fatigue_yeet(data):
	print("Got fatigue yeet data: {}".format(data))
	GPIO.output(stop_pin, GPIO.LOW)
	GPIO.output(forward_pin, GPIO.LOW)
	GPIO.output(pull_over_pin, GPIO.LOW)
	GPIO.output(pump_brake_pin, GPIO.LOW)
	
		

@sio.event
def eyes_yeet(data):
	print("Got eyes on us: {}".format(data))
        # 1 open 0 closed
	if data == 1:
		GPIO.output(forward_pin, GPIO.HIGH)
		GPIO.output(pump_brake_pin, GPIO.LOW)
		GPIO.output(stop_pin, GPIO.LOW)
		GPIO.output(pull_over_pin, GPIO.LOW)
	if data == 0:
		GPIO.output(forward_pin, GPIO.LOW)
		GPIO.output(stop_pin, GPIO.LOW)
		GPIO.output(pull_over_pin, GPIO.LOW)
		GPIO.output(pump_brake_pin, GPIO.HIGH)
	

@sio.event
def attention_yeet(data):
	print("YEET, attention: {}".format(data))
	# 1 aware not aware

sio.connect("http://34.94.143.73:9001")
sio.wait()
