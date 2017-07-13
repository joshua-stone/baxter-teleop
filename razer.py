#!/usr/bin/env python2.7

from rospy import init_node, Subscriber, sleep, spin
from rospy.timer import time
from sensor_msgs.msg import Image, PointCloud
from razer_hydra.msg import Hydra
from baxter_interface import Head, CameraController, Gripper
from lib.window import Window
import rospy

CONTROLLER = '/hydra_calib'
SONAR = '/robot/sonar/head_sonar/state'
LEFT_CAMERA = '/cameras/left_hand_camera/image'
closest_object = None
SENSOR_DATA = []

sensor_locations = {
	3.0: -1.3,
	2.0: -1.0,
	1.0: -0.5,
	12.0: 0.0,
	11.0: 0.5,
	10.0: 1.0,
	9.0: 1.3
}
timestamp = time.time()
#window = Window('left camera')

init_node('Hydra_teleop')

left_gripper, right_gripper = Gripper('left'), Gripper('right')

# Min and max position vary each time
def gripper_calibrate(gripper):
	# A minimum timeout is required so gripper has enough time to move
	# Otherwise an incorrect position will be reported
	WAIT = 0.50
	gripper.calibrate()

	gripper.open()
	rospy.sleep(WAIT)	

	max_position = gripper.position()

	gripper.close()
        rospy.sleep(WAIT)

	min_position = gripper.position()

	gripper.command_position(max_position)

	return (min_position, max_position)

print 'Calibrating left gripper'
left_gripper_min, left_gripper_max = gripper_calibrate(left_gripper)

print 'Calibrating right gripper'
right_gripper_min, right_gripper_max = gripper_calibrate(right_gripper)

print 'left gripper: ', left_gripper_min, left_gripper_max
print 'right_gripper: ', right_gripper_min, right_gripper_max

head = Head()

def camera_setup():
	print 'Setting up head camera'

	head_camera = CameraController('head_camera')
	head_camera.close()

	print 'Setting up left camera'

	left_camera = CameraController('left_hand_camera')
	left_camera.close()

	print 'Setting up right camera'

	right_camera = CameraController('right_hand_camera')
	right_camera.close()

	head_camera.open()

#camera_setup()

#left_camera = CameraController('left_hand_camera')
#right_camera = CameraController('right_hand_camera')

#left_camera.open()

print 'Openning right camera'
#rospy.sleep(5)

#right_camera.open()
print 'starting'

def subscribe(data):
	global SENSOR_DATA, closest_object
	#if head.nodding():
	#	print 'nod'
	left, right = data.paddles
	j = right.joy[0]
	b = right.buttons
	#print b
	ENABLED = b[6]	
	MOVE_HEAD = b[5]
	#print j * -0.0001
	three = b[3]
	one = b[1]
	if ENABLED:
		distance = j * 0.1
		if not -0.01 < j < 0.01:
		#-1.40397596359
		# 1.32229149342
			print distance, j, head.pan()
			head.set_pan(head.pan() - distance)
	else:
		if MOVE_HEAD:
	        	#print SENSOR_DATA
			#print closest_object
			#print 'closest object: ', closest_object[0], sensor_locations[closest_object[0]]
			#print 'moving head..'
			head.set_pan(sensor_locations[closest_object[0]])
	#if j > 0.1:
        #        print distance, head.pan()
                #head.set_pan(head.pan() + 0.0)

def camera_subscribe(data):
	window.display(data)

def sonar(data):
	sensors, distances = data.channels
	global SENSOR_DATA
	global closest_object
	global timestamp

	SENSOR_DATA = zip(sensors.values, distances.values)

	objects_in_view = [sensor for sensor in SENSOR_DATA if sensor[0] in sensor_locations]
	#print 'in view', objects_in_view
	if objects_in_view and time.time() - timestamp>= 3:
		timestamp = time.time()
		closest_object = min(objects_in_view, key=lambda a: a[1])

def main():
	Subscriber(CONTROLLER, Hydra, subscribe)
	Subscriber(SONAR, PointCloud, sonar)
	#Subscriber(LEFT_CAMERA, Image, camera_subscribe)
	spin()

if __name__ == '__main__':
	main()
