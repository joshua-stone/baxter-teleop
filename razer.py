#!/usr/bin/env python2.7

from rospy import init_node, Subscriber, sleep, spin, Time, Duration
from sensor_msgs.msg import Image, PointCloud
from razer_hydra.msg import Hydra
from baxter_interface import Head, CameraController, Gripper, Limb
from lib.baxter import Baxter
from lib.hydracontroller import HydraController
import rospy

CONTROLLER = '/hydra_calib'
SONAR = '/robot/sonar/head_sonar/state'
LEFT_CAMERA = '/cameras/left_hand_camera/image'

LIMB_INTERVAL, SONAR_INTERVAL = Duration(0.05), Duration(3)

closest_object = None
sensor_data = []

sensor_locations = {
	3.0: -1.3,
	2.0: -1.0,
	1.0: -0.5,
	12.0: 0.0,
	11.0: 0.5,
	10.0: 1.0,
	9.0: 1.3
}

init_node('Hydra_teleop')

last_scan = Time.now()
last_nod = Time.now()
last_left_pose = Time.now()
last_right_pose = Time.now()

baxter = Baxter()

print 'left gripper max: ', baxter.left_gripper_max
print 'right_gripper max: ', baxter.right_gripper_max

head = Head()

print 'starting'

def subscribe(data):
	global sensor_data, closest_object, last_nod, last_left_pose, last_right_pose, LIMB_INTERVAL
	# Shift controller coordinates to be higher and back so operator can move comfortably
	controller = HydraController(data, x_offset=0.5, z_offset=-0.5)

	current_time = Time.now()
	current_left_pose = Time.now()
	current_right_pose = Time.now()

	left_joy = controller.left_joy_horizontal
	right_joy = controller.right_joy_horizontal
	if not -0.01 < left_joy < 0.01:
                b.left_w1 += left_joy * 0.1
        if not -0.01 < right_joy < 0.01:
		b.right_w1 += right_joy * 0.1
	'''
	if controller.right_joy_press:
		j = controller.right_joy_horizontal
		distance = j * 0.1
		if not -0.01 < j < 0.01:
		# Min position: -1.40397596359
		# Max position: 1.32229149342
			print distance, j, head.pan()
			b.head_position -= distance

	elif controller.right_middle and closest_object:
		head.set_pan(sensor_locations[closest_object[0]])
	elif controller.right_4 and current_time - last_nod >= 1:
		last_nod = time.time()
		head.command_nod()
	elif controller.right_2 and current_time - last_nod >= 1:
		last_nod = time.time()
		if b.head_position > 0.0:
			distance = 0.25
		else:
			distance = -0.25

		b.head_position -= distance
		rospy.sleep(0.1)
		b.head_position += distance

	else:
		pass
	'''

	if controller.right_4 and current_right_pose - last_right_pose > LIMB_INTERVAL:
		x = controller.right_translation.x
		y = controller.right_translation.y
		z = controller.right_translation.z
		baxter.set_right_pose(position={'x': x, 'y': y, 'z':z})
		last_right_pose = current_right_pose
		print 'right baxter arm:', baxter.right_position
		print 'right controller:', x, y, z
        if controller.left_4 and current_left_pose - last_left_pose > LIMB_INTERVAL:
                x = controller.left_translation.x
                y = controller.left_translation.y
                z = controller.left_translation.z
                baxter.set_left_pose(position={'x': x, 'y': y, 'z':z})
                last_left_pose = current_left_pose
                print 'left baxter arm:', baxter.left_position
                print 'left controller:', x, y, z

	if controller.left_3:
		baxter.left_w2 += 0.1
	if controller.left_1:
		baxter.left_w2 -= 0.1
        if controller.right_3:
		baxter.right_w2 += 0.1
        if controller.right_1:
		baxter.right_w2 -= 0.1
	if controller.left_trigger_1:
                left_pos = baxter.left_gripper_max - (controller.left_trigger_2 * 100.0)
                baxter.left_gripper = left_pos
	if controller.right_trigger_1:
		right_pos = baxter.right_gripper_max - (controller.right_trigger_2 * 100.0)
		baxter.right_gripper = right_pos

def sonar(data):
	global sensor_data, closest_object, last_scan, SONAR_INTERVAL
	sensors, distances = data.channels

	current_time = Time.now()

	sensor_data = zip(sensors.values, distances.values)

	objects_in_view = [sensor for sensor in sensor_data if sensor[0] in sensor_locations]
	if objects_in_view and current_time - last_scan >= SONAR_INTERVAL:
		last_scan = current_time
		closest_object = min(objects_in_view, key=lambda a: a[1])
 
def main():
	Subscriber(CONTROLLER, Hydra, subscribe)
	Subscriber(SONAR, PointCloud, sonar)
	spin()

if __name__ == '__main__':
	main()
