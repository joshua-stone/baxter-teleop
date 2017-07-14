#!/usr/bin/env python2

from baxter_interface import Limb, RobotEnable
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

    def reset_limb(self, side):
        angles = {joint: 0.0 for joint in self._limbs[side].joint_angles()}

        self.enable_check()

        self._limbs[side].move_to_joint_positions(angles)

    def enable_check(self):
        # Sometimes robot is disabled due to another program resetting state
        if not self.enabled:
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
        joints = self._left.joint_angles()
        joints['left_s0'] = angle

        self.enable_check()

        self._left.set_joint_positions(joints)

    @property
    def left_s1(self):
        return self._left.joint_angle('left_s1')

    @left_s1.setter
    def left_s1(self, angle):
        joints = self._left.joint_angles()
        joints['left_s1'] = angle

        self.enable_check()

        self._left.set_joint_positions(joints)

    @property
    def left_e0(self):
        return self._left.joint_angle('left_e0')

    @left_e0.setter
    def left_e0(self, angle):
        joints = self._left.joint_angles()
        joints['left_e0'] = angle

        self.enable_check()

        self._left.set_joint_positions(joints)

    @property
    def left_e1(self):
        return self._left.joint_angle('left_e1')

    @left_e1.setter
    def left_e1(self, angle):
        joints = self._left.joint_angles()
        joints['left_e1'] = angle

        self.enable_check()

        self._left.set_joint_positions(joints)

    @property
    def left_w0(self):
        return self._left.joint_angle('left_w0')

    @left_w0.setter
    def left_w0(self, angle):
        joints = self._left.joint_angles()
        joints['left_w0'] = angle

        self.enable_check()

        self._left.set_joint_positions(joints)

    @property
    def left_w1(self):
        return self._left.joint_angle('left_w1')

    @left_w1.setter
    def left_w1(self, angle):
        joints = self._left.joint_angles()
        joints['left_w1'] = angle

        self.enable_check()

        self._left.set_joint_positions(joints)

    @property
    def left_w2(self):
        return self._left.joint_angle('left_w2')

    @left_w2.setter
    def left_w2(self, angle):
        joints = self._left.joint_angles()
        joints['left_w2'] = angle

        self.enable_check()

        self._left.set_joint_positions(joints)

    @property
    def right_s0(self):
        return self._right.joint_angle('right_s0')

    @right_s0.setter
    def right_s0(self, angle):
        joints = self._right.joint_angles()
        joints['right_s0'] = angle

        self.enable_check()

        self._right.set_joint_positions(joints)

    @property
    def right_s1(self):
        return self._right.joint_angle('right_s1')

    @right_s1.setter
    def right_s1(self, angle):
        joints = self._right.joint_angles()
        joints['right_s1'] = angle

        self.enable_check()

        self._right.set_joint_positions(joints)

    @property
    def right_e0(self):
        return self._right.joint_angle('right_e0')

    @right_e0.setter
    def right_e0(self, angle):
        joints = self._right.joint_angles()
        joints['right_e0'] = angle

        self.enable_check()

        self._right.set_joint_positions(joints)

    @property
    def right_e1(self):
        return self._right.joint_angle('right_e1')

    @right_e1.setter
    def right_e1(self, angle):
        joints = self._right.joint_angles()
        joints['right_e1'] = angle

        self.enable_check()

        self._right.set_joint_positions(joints)

    @property
    def right_w0(self):
        return self._right.joint_angle('right_w0')

    @right_w0.setter
    def right_w0(self, angle):
        joints = self._right.joint_angles()
        joints['right_w0'] = angle

        self.enable_check()

        self._right.set_joint_positions(joints)

    @property
    def right_w1(self):
        return self._right.joint_angle('right_w1')

    @right_w1.setter
    def right_w1(self, angle):
        joints = self._right.joint_angles()
        joints['right_w1'] = angle

        self.enable_check()

        self._right.set_joint_positions(joints)

    @property
    def right_w2(self):
        return self._right.joint_angle('right_w2')    

    @right_w2.setter
    def right_w2(self, angle):
        joints = self._right.joint_angles()
        joints['right_w2'] = angle

        self.enable_check()
        self._right.set_joint_positions(joints)

