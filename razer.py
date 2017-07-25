#!/usr/bin/env python2.7

from rospy import init_node, Subscriber, sleep, spin, Time, Duration
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import Quaternion, Pose
from razer_hydra.msg import Hydra
from baxter_interface import Head, CameraController, Gripper, Limb
from lib.baxter import Baxter
from lib.hydracontroller import HydraController
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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

	left_joy_horizontal = controller.left_joy_horizontal
	right_joy_horizontal = controller.right_joy_horizontal
        left_joy_vertical = controller.left_joy_vertical
        right_joy_vertical = controller.right_joy_vertical

	if not -0.01 < left_joy_horizontal < 0.01 or -0.01 < left_joy_vertical < 0.01:
		w0 = baxter.left_w0 + left_joy_horizontal * 0.1
                w1 = baxter.left_w1 + left_joy_vertical * 0.1
		baxter.set_left_joints({'left_w0': w0, 'left_w1': w1 })

        if not -0.01 < right_joy_horizontal < 0.01 or -0.01 < right_joy_vertical < 0.01:
		w0 = baxter.right_w0 + right_joy_horizontal * 0.1
                w1 = baxter.right_w1 + right_joy_vertical * 0.1

		baxter.set_right_joints({'right_w0': w0, 'right_w1': w1 })
		#baxter.right_w1 += right_joy_horizontal * 0.1

        #if not -0.01 < left_joy_vertical < 0.01:
        #        baxter.left_w1 += left_joy_horizontal * 0.1
        #if not -0.01 < right_joy_vertical < 0.01:
        #        baxter.right_w1 += right_joy_horizonal * 0.1

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
		right_translation, right_rotation = controller.right_translation, controller.right_rotation

		position = {
			'x': right_translation.x,
			'y': right_translation.y,
			'z': right_translation.z
		}
		orientation = {
			'x': right_rotation.x,
			'y': right_rotation.y,
			'z': right_rotation.z,
			'w': right_rotation.w
		}
		print baxter.right_orientation
		#q = Quaternion(right_rotation.x, right_rotation.y, right_rotation.z, right_rotation.w)
		#x, y, z = euler_from_quaternion((right_rotation.x, right_rotation.y, right_rotation.z, right_rotation.w))
		#print 'controller:', x, y, z
		#print 'con offset:', x * 7.6638672291773, y * -2.999, z * 0.3434353738201859
		baxter.set_right_pose(position=position)
		last_right_pose = current_right_pose
		#print 'baxter arm:', euler_from_quaternion(baxter.right_orientation)
		#print ''
		#quart = quaternion_from_euler(x * 7.6638672291773, y * -3, z * 0.3434353738201859)
		#orientation = {
                #        'x': quart[0],
                #        'y': quart[1],
                #        'z': quart[2],
                #        'w': quart[3]
                #}
		#baxter.set_right_pose(position=position, orientation=orientation)
		#print 'right controller:', x, y, z
		#print 'right orientation:', 'x =', right_rotation.x, 'y =', right_rotation.y, 'z =', right_rotation.z, 'w =', right_rotation.w
        if controller.left_4 and current_left_pose - last_left_pose > LIMB_INTERVAL:
                left_translation, left_rotation = controller.left_translation, controller.left_rotation

                position = {
                        'x': left_translation.x,
                        'y': left_translation.y,
                        'z': left_translation.z
                }
                orientation = {
                        'x': left_rotation.x,
                        'y': left_rotation.y,
                        'z': left_rotation.z,
                        'w': left_rotation.w
                }
                baxter.set_left_pose(position=position)
                last_left_pose = current_left_pose
                print 'left baxter arm:', baxter.left_orientation
                print 'left controller:', orientation

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
