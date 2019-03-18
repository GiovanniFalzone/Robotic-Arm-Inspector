#!/usr/bin/python
from math import pi
from robot_motion_lib import *
from copy import deepcopy
import copy
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from robotic_arm_inspector.msg import planes_msg

class train_inspector():
	def __init__(self):
		self.last_Kinect_PC2 = PointCloud2()
		self.init_PC_sender_receiver()
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

	def callback(self, data):
		# print('data received')
		self.last_Kinect_PC2 = copy.deepcopy(data)

	def init_PC_sender_receiver(self):
		self.pub = rospy.Publisher('/robotic_arm_inspector/pad_check', planes_msg, queue_size=10)
		rospy.Subscriber('/camera1/depth/points', PointCloud2, self.callback)

	def send_to_PC_checker(self, pc1, pos1, pc2, pos2):
		msg = planes_msg()
		msg.info = 'check planes distance'
		msg.pc1 = copy.deepcopy(pc1)
		msg.sensor_pos1 = copy.deepcopy(pos1)
		msg.pc2 = copy.deepcopy(pc2)
		msg.sensor_pos2 = copy.deepcopy(pos2)

		print msg.info
		print msg.pc1.fields
		print msg.pc1.header.stamp
		print msg.sensor_pos1
		print msg.pc2.header.stamp
		print msg.sensor_pos2

		rospy.loginfo(msg.info)
		self.pub.publish(msg)

	def move_in_sleep_position(self):
		joints_vect = self.arm_description.get('relax_joints_angles')
		self.motion_lib.set_joint(joints_vect)

	def move_in_waiting_position(self):
		joints_vect = self.arm_description.get('waiting_joints_angles')
		self.motion_lib.set_joint(joints_vect)

	def move_in_checking_position(self):
		pos = self.arm_description.get('checking_position')
		self.motion_lib.move_in_xyz_rpy(pos)

	def inspect_pads_cone(self):
		pads_pos = self.train_description.get('axis').get('pads').get('coordinates')
		dist = self.train_description.get('axis').get('pads').get('disk').get('radius') + 0.4
		print(pads_pos)
		for pos in pads_pos:
			pc1 = copy.deepcopy(self.last_Kinect_PC2)
			pos1 = self.motion_lib.get_pos_xyz_rpy()
			self.motion_lib.follow_cone_base_z(pos, 0.1, dist, 2*math.pi, 1, 5)
			while(pc1.header.stamp == self.last_Kinect_PC2.header.stamp):
				pass
			pc2 = copy.deepcopy(self.last_Kinect_PC2)
			pos2 = self.motion_lib.get_pos_xyz_rpy()
			self.send_to_PC_checker(pc1, pos1, pc2, pos2)
		
	def inspect_pads(self):
		pc1 = copy.deepcopy(self.last_Kinect_PC2)
		pos1 = self.motion_lib.get_pos_xyz_rpy()

		vect_pos = [0.00005, 1.5, 1.75, -math.pi/2, 0, 0]
		self.motion_lib.move_in_xyz_rpy(vect_pos)

		while(pc1.header.stamp == self.last_Kinect_PC2.header.stamp):
			pass
		pc2 = copy.deepcopy(self.last_Kinect_PC2)
		pos2 = self.motion_lib.get_pos_xyz_rpy()
		self.send_to_PC_checker(pc1, pos1, pc2, pos2)

		pc1 = copy.deepcopy(self.last_Kinect_PC2)
		pos1 = self.motion_lib.get_pos_xyz_rpy()

		vect_pos = [0.00005, 1.5, 1.75, 0, 0, 0]
		self.motion_lib.move_in_xyz_rpy(vect_pos)

		while(pc1.header.stamp == self.last_Kinect_PC2.header.stamp):
			pass
		pc2 = copy.deepcopy(self.last_Kinect_PC2)
		pos2 = self.motion_lib.get_pos_xyz_rpy()
		self.send_to_PC_checker(pc1, pos1, pc2, pos2)


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
		pc1 = copy.deepcopy(self.last_Kinect_PC2)
		pos1 = self.motion_lib.get_pos_xyz_rpy()
		self.motion_lib.follow_line(start_pos, direction, n_step, math.pi/2, math.pi/2, 0)
		while(pc1.header.stamp == self.last_Kinect_PC2.header.stamp):
			pass
		pc2 = copy.deepcopy(self.last_Kinect_PC2)
		pos2 = self.motion_lib.get_pos_xyz_rpy()
		self.send_to_PC_checker(pc1, pos1, pc2, pos2)

	def check_train(self, train_desc):
		self.train_description = train_desc
		self.move_in_sleep_position()
		self.move_in_waiting_position()
		self.inspect_axis()
		self.inspect_pads()

