import sys
import socketio

sio = socketio.Client()

@sio.event
def connect():
	print("Connected!")


@sio.event
def disconect():
	print("Got disconnected!")

	# TODO: stop motors
	sys.exit(0)

@sio.event
def fatigue_yeet(data):
	print("Got fatigue yeet data: {}".format(data))

@sio.event
def eyes_yeet(data):
	print("Got eyes on us: {}".format(data))

@sio.event
def attention_yeet(data):
	print("YEET, attention: {}".format(data))

sio.connect("http://34.94.143.73:9001")
sio.wait()
