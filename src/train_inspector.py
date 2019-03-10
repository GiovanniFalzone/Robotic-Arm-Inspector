#!/usr/bin/python
from math import pi
from robot_motion_lib import *

class train_inspector():
	def __init__(self):
		self.motion_lib = robot_motion_lib()
		self.train_description = {
			'axis':{
				'position':[0, 1.5, 1.7],
				'lenght':2,
				'pads':{
					'num':2,
					'coordinates':[
						[0, 1.25, 1.5],
						[0, 1.75, 1.5]
					],
					'disk':{
						'radius': 1,
					},
				}
			}
		}
		self.arm_description = {
			'name':'UR5',
			'num_joints':6,
			'relax_joints_angles':[0,0,0,0,0,0],
			'waiting_joints_angles':[0, -math.pi/4, 0, -math.pi/2, 0, -math.pi/4],
			'checking_position':[0, 1.25, 1.9, math.pi/2, 0, 0]
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
		print(pads_pos)
		for pos in pads_pos:
			self.motion_lib.follow_cone_base_x(pos, 0.1, 0.1, 2*math.pi, 1, 10)
		
	def inspect_axis(self):
		axis_center_pos = self.train_description.get('axis').get('position')
		axis_lenght = self.train_description.get('axis').get('lenght')
		start_pos = axis_center_pos
		start_pos[1] -= axis_lenght/2
		start_pos[2] -=	0.2
		self.motion_lib.follow_line(start_pos, 0, 0.1, 0, 20,  math.pi/2, math.pi/2, 0)

	def check_train(self, train_desc):
		self.train_description = train_desc
		print self.train_description
		self.move_in_sleep_position()
		self.move_in_waiting_position()
		self.inspect_axis()
		self.inspect_pads()

