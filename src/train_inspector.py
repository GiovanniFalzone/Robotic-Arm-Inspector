#!/usr/bin/python
from math import pi
from robot_motion_lib import *
from copy import deepcopy

class train_inspector():
	def __init__(self):
		self.motion_lib = robot_motion_lib()
		self.train_description = {
			'axis':{
				'position':[0, 1.5, 2],
				'lenght':2,
				'pads':{
					'num':2,
					'coordinates': [[0.0, 1.2, 2.0], [0.0, 1.8, 2.0]],
					'disk':{
						'radius': 0.25,
					},
				}
			}
		}
		self.arm_description = {
			'name':'UR5',
			'num_joints':6,
			'relax_joints_angles':[0,0,0,0,0,0],
			'waiting_joints_angles':[0, -math.pi/4, 0, -math.pi/2, 0, -math.pi/4],
			'checking_position':[0, 1.5, 1.5, math.pi/2, 0, 0]
		}

	def move_in_sleep_position(self):
		joints_vect = self.arm_description.get('relax_joints_angles')
		self.motion_lib.set_joint(joints_vect)

	def move_in_waiting_position(self):
		joints_vect = self.arm_description.get('waiting_joints_angles')
		self.motion_lib.set_joint(joints_vect)

	def move_in_checking_position(self):
		pos = self.arm_description.get('checking_position')
		self.motion_lib.move_in_xyz_rpy(pos)

	def inspect_pads(self):
		pads_pos = self.train_description.get('axis').get('pads').get('coordinates')
		dist = self.train_description.get('axis').get('pads').get('disk').get('radius') + 0.4
		print(pads_pos)
		for pos in pads_pos:
			self.motion_lib.follow_cone_base_z(pos, 0.1, dist, 2*math.pi, 1, 5)
		
	def inspect_axis(self):
		axis_center_pos = deepcopy(self.train_description.get('axis').get('position'))
		axis_lenght = self.train_description.get('axis').get('lenght')
		dist = self.train_description.get('axis').get('pads').get('disk').get('radius') + 0.25
		print axis_center_pos
		start_pos = axis_center_pos
		start_pos[1] -= axis_lenght/2
		start_pos[2] -= dist
		print axis_center_pos
		n_step = 5
		step = (float(axis_lenght)/float(n_step))
		direction = [0, step, 0]
		self.motion_lib.follow_line(start_pos, direction, n_step, math.pi/2, math.pi/2, 0)

	def check_train(self, train_desc):
		self.train_description = train_desc
		self.move_in_sleep_position()
		self.move_in_waiting_position()
		self.inspect_axis()
		self.inspect_pads()

