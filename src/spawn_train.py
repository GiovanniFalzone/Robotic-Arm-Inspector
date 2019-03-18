#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SpawnModelRequest, SpawnModelResponse	
from std_msgs.msg import String													# to exchange string messages
from copy import deepcopy
from xml.dom import minidom														# to parse sdf model
from math import pi
import json
import random
import ast

from train_inspector import *
from train_model_parser import *

train_model = train_model()				# parser to work on train sdf and train dictionary
motion_lib = robot_motion_lib()			# motion lib for ur5
inspector = train_inspector()			# train inspector to "visit" the train

# global variables
x0 = 0.0
y0 = 0.0
z0 = 2.0
model_id = 0

# a pad is considered "good" or "bad" w.r.t. its width
good_pad_width = 0.1		
bad_pad_width = 0.02
high_treshold = (good_pad_width - bad_pad_width)* 2 / 3
mid_treshold = (good_pad_width - bad_pad_width) / 2
low_treshold = (good_pad_width - bad_pad_width) / 3

# given a pad width we need to find its center related to disks positions
def find_pad_position(dir, pw):
	"""Calculates pad position for gazebo, with respect to its width
	and the disks position as: disk_center +- half_disk +- pad width"""
	y = 0

	axis_width = train_model.train_struct_get_width('axis')
	disk_width = train_model.train_struct_get_width('disks')

	if "left" in dir:
		y = axis_width / 3 + disk_width / 2
	elif "right" in dir:
		y = 2*axis_width/3 - disk_width / 2 - pw	
	
	pose_str = train_model.xyz_to_pose_string(x0, y, z0)
	return pose_str

# manage pads width in sdf and to spawn the train
def set_pads_width(left_pad_width, right_pad_width):
	"""Modify pads width in train_structure and in sdf model, to show on gazebo"""
	links = train_model.sdf_get_links()

	train_model.train_struct_set_width('pads', left_pad_width, 'left')
	train_model.train_struct_set_width('pads', right_pad_width, 'right')

	#iterate on links
	for elem in links:
		# check if name attribute is "left_pad" or "right_pad"
		if train_model.sdf_check_name_attribute(elem, "left_pad"):
			# get new position coordinates for left pad
			pad_pose_value = find_pad_position("left", left_pad_width)
				
			# get "pose" element from sdf and modify its value
			pad_pose = train_model.sdf_get_pose(elem)
			for pose in pad_pose:
				train_model.sdf_set_inner_value(pose, pad_pose_value)
			
			# get "length" element from sdf and modify its value
			pad_width = train_model.sdf_get_height(elem)
			for width in pad_width:
				train_model.sdf_set_inner_value(width, left_pad_width)

			if(left_pad_width > low_treshold):
				if(left_pad_width > high_treshold):
					train_model.sdf_set_pad_color(elem, "Green")
				else:
					train_model.sdf_set_pad_color(elem, "Yellow")
			else:
				train_model.sdf_set_pad_color(elem, "Red")

			train_model.train_struct_set_left_pose('pads', pad_pose_value)
			
		elif train_model.sdf_check_name_attribute(elem, "right_pad"):
			pad_pose_value = find_pad_position("right", right_pad_width)
				
			pad_pose = train_model.sdf_get_pose(elem)
			for pose in pad_pose:
				train_model.sdf_set_inner_value(pose, pad_pose_value)
			
			pad_width = train_model.sdf_get_height(elem)
			for width in pad_width:
				train_model.sdf_set_inner_value(width, right_pad_width)

			if(right_pad_width > low_treshold):
				if(right_pad_width > high_treshold):
					train_model.sdf_set_pad_color(elem, "Green")
				else:
					train_model.sdf_set_pad_color(elem, "Yellow")
			else:
				train_model.sdf_set_pad_color(elem, "Red")

			train_model.train_struct_set_right_pose('pads', pad_pose_value)

def create_train_request(modelname, px, py, pz, lpw, rpw):
	"""Create a SpawnModelRequest with the parameters of the train given.
	modelname: name of the model for gazebo
	px py pz: position of the train (and it's collision train)
	lpw: left pad width
	rpw: right pad width"""
	
	set_pads_width(lpw,rpw)
	train_model.train_struct_set_name(modelname)

	train = deepcopy(train_model.get_sdf())

	req = SpawnModelRequest()
	req.model_name = modelname
	req.model_xml = train
	req.initial_pose.position.x = px
	req.initial_pose.position.y = py
	req.initial_pose.position.z = pz

	return req

def spawn_train(spawn_srv, delete_srv):
	"""Interacts with user to spawn a train, specifying pads specs. It requests
	spawn_srv: the ros proxy to spawn model on gazebo
	delete_srv: the ros proxy to delete model from gazebo"""
	
	out_msg = 'Press \n\
\t-> 1 to spawn a train with "good" pads \n\
\t-> 2 to spawn a train with "bad" pads \n\
\t-> 3 to spawn a train with random pads \n\
\t-> 4 to spawn a train with chosen pads width \n\
\t-> c to continue (train already spawned)\n'

	global model_id

	in_cmd = raw_input(out_msg)
	model_id +=1
	if '1' in in_cmd:
		# Spawn train with good pads
		print "Spawning train with good pads"
		req = create_train_request("train_model_"+str(model_id), 		# model name
									0.0, 0.0, 0.0,		# train initial position
									good_pad_width, good_pad_width)  		# pads length (status)
		spawn_srv.call(req)
	elif '2' in in_cmd:
		print "Spawning train with bad pads"
		req = create_train_request("train_model_"+str(model_id), 
									0.0, 0.0, 0.0,
									bad_pad_width, bad_pad_width) 
		spawn_srv.call(req)
	elif '3' in in_cmd:
		print "Spawning train with random pads width"
		lpw = random.uniform(bad_pad_width, good_pad_width)
		print "-left pad width "+str(lpw)
		rpw = random.uniform(bad_pad_width, good_pad_width)
		print "-right pad width "+str(rpw)
		req = create_train_request("train_model_"+str(model_id), 
									0.0, 0.0, 0.0,
									lpw, rpw) 
		spawn_srv.call(req)
	elif '4' in in_cmd:
		out_msg = 'Choose pads width for left pad\n'
		lpw = float(raw_input(out_msg))
		out_msg = 'Choose pads width for right pad\n'
		rpw = float(raw_input(out_msg))
		print "Spawning train with chosen pads width:"
		print "-left pad width "+str(lpw)
		print "-right pad width "+str(rpw)
		req = create_train_request("train_model_"+str(model_id),
									0.0, 0.0, 0.0,
									lpw, rpw)
		spawn_srv.call(req)	
	elif 'c' in in_cmd:
		print train_model.train_structure
		return 1	

	while True:
		conf_msg = 'Press \n\
	\t-> 0 to delete the spawned train \n\
	\t-> p to print train specs \n\
	\t-> c to confirm and continue \n'
		in_cmd = raw_input(conf_msg)

		if '0' in in_cmd:
			print "Deleting train model"
			delete_srv("train_model_"+str(model_id))
			return 0
		elif 'p' in in_cmd:
			train_model.train_struct_print()
		else:
			return 1

def get_number(msg):
	text = raw_input(msg)
	return eval(text)

def get_vector(msg):
	text = raw_input(msg + ' ([x,y,z]):')
	coords = text.split(']')[0].split('[')[1].split(',')
	x = eval(coords[0])
	y = eval(coords[1])
	z = eval(coords[2])
	return [x, y, z]

def get_position():
	return get_vector('Insert coordinates')

def inspect_train():
	inspector.set_train_description(train_model.train_structure)

	msg = 'Press \n\
\t-> 1 to move in initial position \n\
\t-> 2 to move in max estension position \n\
\t-> 3 to move in checking position \n\
\t-> line to move following a cone base \n\
\t-> circle to move following a circle \n\
\t-> cone to move in checking position \n\
\t-> axis to check axis\n\
\t-> pad to check pads\n\
\t-> check to check the train\n'
	cmd = raw_input(msg)
	if('1' in cmd):
		inspector.move_in_sleep_position()
	elif('2' in cmd):
		inspector.move_in_waiting_position()
	elif('3' in cmd):
		inspector.move_in_checking_position()
	elif('circle' in cmd):
		axis = raw_input('which axis?(x, y or z)') 
		vect_pos = get_position()
		print('Circle center: ' + str(vect_pos))
		if('x' in axis):
			motion_lib.follow_circle_x(vect_pos)
		elif('y' in axis):
			motion_lib.follow_circle_y(vect_pos)
		elif('z' in axis):
			motion_lib.follow_circle_z(vect_pos)

	elif('cone' in cmd):
		axis = raw_input('which axis?(x, y or z)') 
		vect_pos = get_position()
		print('Cone vertex: ' + str(vect_pos))
		if('x' in axis):
			motion_lib.follow_cone_base_x(vect_pos)
		elif('y' in axis):
			motion_lib.follow_cone_base_y(vect_pos)
		elif('z' in axis):
			motion_lib.follow_cone_base_z(vect_pos)

	elif('line' in cmd):
		vect_pos = get_position()
		steps = get_number('Insert How many step:')
		direction = get_vector('Insert Direction (Delta for each axis)')
		motion_lib.follow_line(vect_pos, direction, steps)
	elif('train_axis' in cmd):
		inspector.inspect_axis()
	elif('train_pads' in cmd):
		inspector.inspect_pads()
	elif('pos' in cmd):
		vect_pos = get_position()
		vect_pos.extend([0,0,0]),
		print('moving to: ' + str(vect_pos))
		motion_lib.move_in_xyz_rpy(vect_pos)

	elif('rotate' in cmd):
		vect = get_vector('Insert rpy as: ')
		motion_lib.rotate(vect)

	elif('pad' in cmd):
		inspector.inspect_pads()

	elif('axis' in cmd):
		inspector.inspect_axis()

	elif('check' in cmd):
		inspector.check_train(train_model.train_structure)

	else:
		print('Wrong input \n')

if __name__ == '__main__':

	# connection to spawn_sdf_model service and to delete_model service of gazebo
	#rospy.init_node('spawn_models', anonymous=True)
	delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
	rospy.loginfo("Waiting for /gazebo/delete_model service...")
	delete_srv.wait_for_service()
	rospy.loginfo("Connected to delete service!")

	spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
	spawn_srv.wait_for_service()
	rospy.loginfo("Connected to spawn service!")

	cont = 0
	while cont == 0:
		try:
			cont = spawn_train(spawn_srv, delete_srv)
		except rospy.ROSInterruptException:
			print('rospy exception')
			break
		except KeyboardInterrupt:
			print('keyboard interrupt')
			break
		
	while True:
		try:
			inspect_train()
		except rospy.ROSInterruptException:
			print('rospy exception')
			break
		except KeyboardInterrupt:
			print('keyboard interrupt')
			break
