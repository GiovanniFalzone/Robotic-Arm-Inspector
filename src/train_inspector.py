#!/usr/bin/python
from math import pi
from robot_motion_lib import *

class train_inspector():
	def __init__(self):
		self.motion_lib = robot_motion_lib()
		self.pads=[[0, 1.33, 1.7], [0, 2.66, 1.7] ]

	def move_in_sleep_position(self):
		self.motion_lib.set_joint(0, 0, 0, 0, 0, 0)

	def move_in_waiting_position(self):
		self.motion_lib.set_joint(0, -math.pi/4, 0, -math.pi/2, 0, -math.pi/4)

	def move_in_checking_position(self):
		self.motion_lib.move_in_xyz_rpy(0, 0.25, 1.9, math.pi/2, 0, 0)

	def inspect_pads(self):
		for pad in self.pads:
			self.motion_lib.follow_cone_base_x(pad[0], pad[1], pad[2], 0.2, 0.2, math.pi, 1, 10)


