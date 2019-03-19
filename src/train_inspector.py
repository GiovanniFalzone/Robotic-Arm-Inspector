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
			'name' : 'train_model_name',
			'axis' : {
				'pose' : 'x y z roll pitch yaw',
				'coordinates' : [0.0, 0.0, 0.0],
				'radius' : 'radius',
				'width': 'width',
			},
			'wheels' : {
				'name' : 'wheels',
				'pose' : { 'left': 'lx ly lz lroll lpitch lyaw', 'right' : 'rx ry rz rroll rpitch ryaw'},
				'coordinates' : {'left' : [0.0, 0.0, 0.0], 'right': [0.0, 0.0, 0.0]},
				'radius' : 'wheels radius',
				'width': 'wheels width',
			},
			'disks' : {
				'name' : 'disks',
				'pose' : { 'left': 'lx ly lz lroll lpitch lyaw', 'right' : 'rx ry rz rroll rpitch ryaw'},
				'coordinates' : {'left' : [0.0, 0.0, 0.0], 'right': [0.0, 0.0, 0.0]},
				'radius' : 'disks radius',
				'width': 'disks width',
			},
			'pads' : {
				'name' : 'pads',
				'pose' : { 'left': 'lx ly lz lroll lpitch lyaw', 'right' : 'rx ry rz rroll rpitch ryaw'},
				'coordinates' : {'left' : [0.0, 0.0, 0.0], 'right': [0.0, 0.0, 0.0]},
				'width': {'left': 'left width', 'right': 'right_width'},
			}
		}
		
		self.arm_description = {
			'name':'UR5',
			'num_joints':6,
			'relax_joints_angles':[0,0,0,0,0,0],
			'waiting_joints_angles':[0, -math.pi/4, 0, -math.pi/2, 0, -math.pi/4],
			'checking_position':[0, 1.5, 1.5, math.pi/2, 0, 0]
		}

	def set_train_description(self, train_desc):
		self.train_description = train_desc
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
		left_pad = self.train_description.get('pads').get('coordinates').get('left')
		right_pad = self.train_description.get('pads').get('coordinates').get('right')
		dist = self.train_description.get('disks').get('radius') + 0.4
		self.motion_lib.follow_cone_base_z(left_pad, 0.1, dist, 2*math.pi, 1, 5)
		self.motion_lib.follow_cone_base_z(right_pad, 0.1, dist, 2*math.pi, 1, 5)
		
	def inspect_pads(self):
		pc1 = copy.deepcopy(self.last_Kinect_PC2)
		pos1 = self.motion_lib.get_pos_xyz_rpy()

		vect_pos = [0.1, 1.5, 1.8, -math.pi/2, 0, 0]
		self.motion_lib.move_in_xyz_rpy(vect_pos)

		while(pc1.header.stamp == self.last_Kinect_PC2.header.stamp):
			pass
		pc2 = copy.deepcopy(self.last_Kinect_PC2)
		pos2 = self.motion_lib.get_pos_xyz_rpy()
		self.send_to_PC_checker(pc1, pos1, pc2, pos2)

		pc1 = copy.deepcopy(self.last_Kinect_PC2)
		pos1 = self.motion_lib.get_pos_xyz_rpy()

		vect_pos = [0, 1.4, 1.75, 0, 0, 0]
		self.motion_lib.move_in_xyz_rpy(vect_pos)

		while(pc1.header.stamp == self.last_Kinect_PC2.header.stamp):
			pass
		pc2 = copy.deepcopy(self.last_Kinect_PC2)
		pos2 = self.motion_lib.get_pos_xyz_rpy()
		self.send_to_PC_checker(pc1, pos1, pc2, pos2)


	def inspect_axis(self):
		axis_center_pos = deepcopy(self.train_description.get('axis').get('coordinates'))
		axis_lenght = self.train_description.get('axis').get('width')
		dist = self.train_description.get('disks').get('radius') + 0.25
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
		self.move_in_sleep_position()
		self.move_in_waiting_position()
		self.explore_world()
		self.inspect_pads()

	def explore_world(self):
		# move under train and apply cone inspection
		axis_center_pos = deepcopy(self.train_description.get('axis').get('coordinates'))
		axis_lenght = self.train_description.get('axis').get('width')
		dist = self.train_description.get('disks').get('radius') + 0.5
		start_pos = axis_center_pos
		axis_border_offset = axis_lenght/3
		start_pos[1] -= axis_border_offset
		start_pos[2] -= dist
		n_step = 3
		step = (float(axis_lenght-2*axis_border_offset)/float(n_step))

		sensor_dir = [math.pi/2, math.pi/2, math.pi/2]

		#check first half plane, joint is limited to move within pi
		for i in range(0,5):
			joints_vect = [0, -i*math.pi/4, 0, -math.pi/2, 0, 0]
			self.motion_lib.set_joint(joints_vect)

		#check second half plane
		for i in range(0,5):
			joints_vect = [0, i*math.pi/4, 0, -math.pi/2, 0, 0]
			self.motion_lib.set_joint(joints_vect)

		# now in a smaller concentric circle
		#check first half plane, joint is limited to move within pi
		for i in range(0,5):
			joints_vect = [0, -i*math.pi/4, -math.pi/2, -math.pi/2, 0, 0]
			self.motion_lib.set_joint(joints_vect)

		#check second half plane
		for i in range(0,5):
			joints_vect = [0, i*math.pi/4, -math.pi/2, -math.pi/2, 0, 0]
			self.motion_lib.set_joint(joints_vect)


		self.move_in_sleep_position()

		#check under axis
		for i in range(0, n_step):
			x = start_pos[0]
			y = start_pos[1] + i*step
			z = start_pos[2]
			pos = [x, y, z]
			pos.extend(sensor_dir)
			self.motion_lib.move_in_xyz_rpy(pos)

		#check pads
		self.inspect_pads_cone()

