#!/usr/bin/env python2

from baxter_interface import Limb, Head, RobotEnable
from geometry_msgs.msg import Pose, Point, Quaternion
from lib.ikservice import IKService
from rospy import init_node

__all__ = [
    'Baxter',
    'LEFT',
    'RIGHT'
]

LEFT, RIGHT = 'left', 'right'

class Baxter(object):
    def __init__(self):
 
        self._baxter_state = RobotEnable()

        self._left = Limb(LEFT)
        self._right = Limb(RIGHT)

        self._limbs = {
            LEFT: self._left,
            RIGHT: self._right
        }

	self._head = Head()
        self._left_ikservice = IKService(LEFT)
        self._right_ikservice = IKService(RIGHT)

    def set_left_joints(self, angles):
	joints = self._left.joint_angles()

	for joint, angle in angles.iteritems():
		if angle:
			joints[joint] = angle

	self.enable_check()
	self._left.set_joint_positions(joints)

    def set_right_joints(self, angles):
        joints = self._right.joint_angles()

        for joint, angle in angles.iteritems():
                if angle:
                        joints[joint] = angle

        self.enable_check()
        self._right.set_joint_positions(joints)	

    def reset_limb(self, side):
        angles = {joint: 0.0 for joint in self._limbs[side].joint_angles()}

        self.enable_check()

        self._limbs[side].move_to_joint_positions(angles)

    def enable_check(self):
        # Sometimes robot is disabled due to another program resetting state
        if not self._baxter_state.state().enabled:
            self._baxter_state.enable()

    @property
    def joints(self):
        joints = {limb: joint.joint_angles() for limb, joint in self._limbs.iteritems()}
        return joints
    @property
    def enabled(self):
        return self._baxter_state.state().enabled

    @property
    def left_s0(self):
        return self._left.joint_angle('left_s0')

    @left_s0.setter
    def left_s0(self, angle):
        self.set_left_joints({'left_s0': angle})

    @property
    def left_s1(self):
        return self._left.joint_angle('left_s1')

    @left_s1.setter
    def left_s1(self, angle):
	self.set_left_joints({'left_s1': angle})

    @property
    def left_e0(self):
        return self._left.joint_angle('left_e0')

    @left_e0.setter
    def left_e0(self, angle):
	self.set_left_joints({'left_e0': angle})

    @property
    def left_e1(self):
        return self._left.joint_angle('left_e1')

    @left_e1.setter
    def left_e1(self, angle):
	self.set_left_joints({'left_e1': angle})

    @property
    def left_w0(self):
        return self._left.joint_angle('left_w0')

    @left_w0.setter
    def left_w0(self, angle):
	self.set_left_joints({'left_w0': angle})
    @property
    def left_w1(self):
        return self._left.joint_angle('left_w1')

    @left_w1.setter
    def left_w1(self, angle):
	self.set_left_joints({'left_w1': angle})

    @property
    def left_w2(self):
        return self._left.joint_angle('left_w2')

    @left_w2.setter
    def left_w2(self, angle):
	self.set_left_joints({'left_w2': angle})

    @property
    def right_s0(self):
        return self._right.joint_angle('right_s0')

    @right_s0.setter
    def right_s0(self, angle):
	self.set_right_joints({'right_s0': angle})

    @property
    def right_s1(self):
        return self._right.joint_angle('right_s1')

    @right_s1.setter
    def right_s1(self, angle):
	self.set_right_joints({'right_s1': angle})

    @property
    def right_e0(self):
        return self._right.joint_angle('right_e0')

    @right_e0.setter
    def right_e0(self, angle):
	self.set_right_joints({'right_e0': angle})

    @property
    def right_e1(self):
        return self._right.joint_angle('right_e1')

    @right_e1.setter
    def right_e1(self, angle):
	self.set_right_joints({'right_e1': angle})

    @property
    def right_w0(self):
        return self._right.joint_angle('right_w0')

    @right_w0.setter
    def right_w0(self, angle):
	self.set_right_joints({'right_w0': angle})

    @property
    def right_w1(self):
        return self._right.joint_angle('right_w1')

    @right_w1.setter
    def right_w1(self, angle):
	self.set_right_joints({'right_w1': angle})

    @property
    def right_w2(self):
        return self._right.joint_angle('right_w2')    

    @right_w2.setter
    def right_w2(self, angle):
	self.set_right_joints({'right_w2': angle})

    @property
    def left_position(self):
        return self._left.endpoint_pose()['position']

    @property
    def left_position_x(self):
        return self.left_position.x

    @left_position_x.setter
    def left_position_x(self, point):
        self.set_left_pose(position={'x': point})

    @property
    def left_position_y(self):
        return self.left_position.y

    @left_position_y.setter
    def left_position_y(self, point):
        self.set_left_pose(position={'y': point})

    @property
    def left_position_z(self):
        return self.left_position.z

    @left_position_z.setter
    def left_position_z(self, point):
        self.set_left_pose(position={'z': point})

    @property
    def left_orientation(self):
        return self._left.endpoint_pose()['orientation']

    @property
    def left_orientation_x(self):
        return self.left_orientation.x

    @left_orientation_x.setter
    def left_orientation_x(self, point):
        self.set_left_pose(orientation={'x': point})

    @property
    def left_orientation_y(self):
        return self.left_orientation.y

    @left_orientation_y.setter
    def left_orientation_y(self, point):
        self.set_left_pose(orientation={'y': point})

    @property
    def left_orientation_z(self):
        return self.left_orientation.z

    @left_orientation_z.setter
    def left_orientation_z(self, point):
        self.set_left_pose(orientation={'z': point})

    @property
    def left_orientation_w(self):
        return self.left_orientation.w

    @left_orientation_w.setter
    def left_orientation_w(self, point):
        self.set_left_pose(orientation={'w': point})

    @property
    def right_position(self):
        return self._right.endpoint_pose()['position']

    @property
    def right_orientation(self):
        return self._right.endpoint_pose()['orientation']

    def set_left_pose(self, position={}, orientation={}):
        pos = {
            'x': self.left_position_x,
            'y': self.left_position_y,
            'z': self.left_position_z,
        }
        for key, value in position.iteritems():
            pos[key] = value

	orient = {
            'x': self.left_orientation_x,
            'y': self.left_orientation_y,
            'z': self.left_orientation_z,
            'w': self.left_orientation_w,
        }
        for key, value in orientation.iteritems():
            orient[key] = value


        pos = self._left_ikservice.solve_position(Pose(position=Point(**pos),
                                                  orientation=Quaternion(**orient)))

	if pos:
            self.set_left_joints(pos)
	else:
            print 'nothing'

	#print self.joints

    @property
    def right_position(self):
        return self._right.endpoint_pose()['position']

    @property
    def right_position_x(self):
        return self.right_position.x

    @right_position_x.setter
    def right_position_x(self, point):
        self.set_right_pose(position={'x': point})

    @property
    def right_position_y(self):
        return self.right_position.y

    @right_position_y.setter
    def right_position_y(self, point):
        self.set_right_pose(position={'y': point})

    @property
    def right_position_z(self):
        return self.right_position.z

    @right_position_z.setter
    def right_position_z(self, point):
        self.set_right_pose(position={'z': point})

    @property
    def right_orientation(self):
        return self._right.endpoint_pose()['orientation']

    @property
    def right_orientation_x(self):
        return self.right_orientation.x

    @right_orientation_x.setter
    def right_orientation_x(self, point):
        self.set_right_pose(orientation={'x': point})

    @property
    def right_orientation_y(self):
        return self.right_orientation.y

    @right_orientation_y.setter
    def right_orientation_y(self, point):
        self.set_right_pose(orientation={'y': point})

    @property
    def right_orientation_z(self):
        return self.right_orientation.z

    @right_orientation_z.setter
    def right_orientation_z(self, point):
        self.set_right_pose(orientation={'z': point})

    @property
    def right_orientation_w(self):
        return self.right_orientation.w

    @right_orientation_w.setter
    def right_orientation_w(self, point):
        self.set_right_pose(orientation={'w': point})

    @property
    def right_position(self):
        return self._right.endpoint_pose()['position']

    @property
    def right_orientation(self):
        return self._right.endpoint_pose()['orientation']

    def set_right_pose(self, position={}, orientation={}):
        pos = {
            'x': self.right_position_x,
            'y': self.right_position_y,
            'z': self.right_position_z,
        }
        for key, value in position.iteritems():
            pos[key] = value

        orient = {
            'x': self.right_orientation_x,
            'y': self.right_orientation_y,
            'z': self.right_orientation_z,
            'w': self.right_orientation_w,
        }
        for key, value in orientation.iteritems():
            orient[key] = value


        pos = self._right_ikservice.solve_position(Pose(position=Point(**pos),
                                                  orientation=Quaternion(**orient)))

        if pos:
            self.set_right_joints(pos)

    @property
    def head_position(self):
        return self._head.pan()

    @head_position.setter
    def head_position(self, position):
	self._head.set_pan(position)
