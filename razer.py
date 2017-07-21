#!/usr/bin/env python2.7

from rospy import init_node, Subscriber, sleep, spin
from rospy.timer import time
from sensor_msgs.msg import Image, PointCloud
from razer_hydra.msg import Hydra
from baxter_interface import Head, CameraController, Gripper, Limb
from lib.window import Window
from lib.baxter import Baxter
from lib.hydracontroller import HydraController
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
last_nod = time.time()
last_left_pose = time.time()
last_right_pose = time.time()
init_node('Hydra_teleop')

b = Baxter()
left_gripper, right_gripper = Gripper('left'), Gripper('right')

# Min and max position vary each time
def gripper_calibrate(gripper):
	# A minimum timeout is required so gripper has enough time to move
	# Otherwise an incorrect position will be reported
	WAIT = 0.5
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

#left_arm = Limb('left')
#right_arm = Limb('right')


#left_joint_angles = left_arm.joint_angles()

#print left_arm.joint_angle('left_w2')

#left_joint_angles['left_w2'] = 0
#print left_joint_angles['left_w2']
#rospy.sleep(2)
#left_arm.set_joint_positions(left_joint_angles)
#rospy.sleep(5)
#print left_arm.joint_angle('left_w2')

#rospy.sleep(2)
#right_joint_angles = right_arm.joint_angles()
#right_joint_angles['right_w2'] = 0
#right_arm.set_joint_positions(right_joint_angles)

#rospy.sleep(2)
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
	controller = HydraController(data)
	global SENSOR_DATA, closest_object, last_nod, last_left_pose, last_right_pose
	current_time = time.time()
	current_left_pose = time.time()
	current_right_pose = time.time()

	if controller.right_joy_press:
		j = controller.right_joy_horizontal
		distance = j * 0.1
		if not -0.01 < j < 0.01:
		#-1.40397596359
		# 1.32229149342
			print distance, j, head.pan()
			b.head_position -= distance

	elif controller.right_middle and closest_object:
		head.set_pan(sensor_locations[closest_object[0]])
	#elif controller.right_4 and current_time - last_nod >= 1:
	#	last_nod = time.time()
	#	head.command_nod()
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
	if controller.right_4 and current_right_pose - last_right_pose > 0.1:
		x = controller.right_translation.x + 0.5
		y = controller.right_translation.y
		z = controller.right_translation.z - 0.5
		b.set_right_pose(position={'x': x, 'y': y, 'z':z})
		last_right_pose = current_right_pose
		print 'right baxter arm:', b.right_position
		print 'right controller:', x, y, z
        if controller.left_4 and current_left_pose - last_left_pose > 0.1:
                x = controller.left_translation.x + 0.5
                y = controller.left_translation.y
                z = controller.left_translation.z - 0.5
                b.set_left_pose(position={'x': x, 'y': y, 'z':z})
                last_left_pose = current_left_pose
                print 'left baxter arm:', b.left_position
                print 'left controller:', x, y, z

	if controller.left_3:
		b.left_w2 += 0.1
	if controller.left_1:
		b.left_w2 -= 0.1
        if controller.right_3:
		b.right_w2 += 0.1
        if controller.right_1:
		b.right_w2 -= 0.1
	if controller.left_trigger_1:
                left_pos = left_gripper_max - (controller.left_trigger_2 * 100.0)
                left_gripper.command_position(left_pos)
	if controller.right_trigger_1:
		right_pos = right_gripper_max - (controller.right_trigger_2 * 100.0)
		right_gripper.command_position(right_pos)


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
