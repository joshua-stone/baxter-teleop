class HydraController(object):
	def __init__(self, data, x_offset=0.0, y_offset=0.0, z_offset=0.0):
		self._left, self._right = data.paddles
		self._lbuttons, self._rbuttons = self._left.buttons, self._right.buttons

		self._ljoy, self._rjoy = self._left.joy, self._right.joy

		self._ltrigger, self._rtrigger = self._left.trigger, self._right.trigger

		self._ltransform, self._rtransform = self._left.transform, self._right.transform

		self._ltransform.translation.x += x_offset
		self._ltransform.translation.y += y_offset
		self._ltransform.translation.z += z_offset

		self._rtransform.translation.x += x_offset
                self._rtransform.translation.y += y_offset
                self._rtransform.translation.z += z_offset

        @property
        def left_trigger_1(self):
                return self._lbuttons[0]

	@property
	def left_1(self):
		return self._lbuttons[1]

        @property
        def left_2(self):
                return self._lbuttons[2]

        @property
        def left_3(self):
                return self._lbuttons[3]

        @property
        def left_4(self):
                return self._lbuttons[4]

        @property
        def left_middle(self):
                return self._lbuttons[5]

        @property
        def left_joy_press(self):
                return self._lbuttons[6]

	@property
	def left_joy_horizontal(self):
		return self._ljoy[0]

        @property
        def left_joy_vertical(self):
                return self._ljoy[1]

        @property
        def left_trigger_2(self):
                return self._ltrigger

        @property
        def right_trigger_1(self):
                return self._rbuttons[0]

        @property
        def right_1(self):
                return self._rbuttons[1]

        @property
        def right_2(self):
                return self._rbuttons[2]

        @property
        def right_3(self):
                return self._rbuttons[3]

        @property
        def right_4(self):
                return self._rbuttons[4]

        @property
        def right_middle(self):
                return self._rbuttons[5]

        @property
        def right_joy_press(self):
                return self._rbuttons[6]

        @property
        def right_joy_horizontal(self):
                return self._rjoy[0]

        @property
        def left_joy_vertical(self):
                return self._rjoy[1]

        @property
        def right_trigger_2(self):
                return self._rtrigger

	@property
	def left_translation(self):
		return self._ltransform.translation

	@property
	def left_rotation(self):
		return self._ltransform.rotation

        @property
        def right_translation(self):
                return self._rtransform.translation

        @property
        def right_rotation(self):
                return self._rtransform.rotation
