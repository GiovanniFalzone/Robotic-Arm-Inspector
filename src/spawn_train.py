#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel, SpawnModelRequest, SpawnModelResponse	
from std_msgs.msg import String													# to exchange string messages
from copy import deepcopy
from xml.dom import minidom														# to parse sdf model
import json
import random

from train_model_parser import *

train_model = train_model()

# global variables
x0 = 0.0
y0 = 0.0
z0 = 2.0

# a pad is considered "good" or "bad" w.r.t. its width
good_pad_width = 0.1		
bad_pad_width = 0.05

# given a pad width we need to find its center related to disks positions
def find_pad_position(dir, pw):
	y = 0

	axis_width = train_model.train_struct_get_width("axis")
	disk_width = train_model.train_struct_get_width("disks")

	if "left" in dir:
		y = axis_width / 3 + disk_width / 2
	elif "right" in dir:
		y = 2*axis_width/3 - disk_width / 2 - pw	
	
	pose_str = train_model.xyz_to_pose_string(x0, y, z0)
	return pose_str

# manage pads width in sdf and to spawn the train
def set_pads_width(left_pad_width, right_pad_width):
	links = train_model.sdf_get_links()

	train_model.train_struct_set_pose("pads", [left_pad_width, right_pad_width])

	#iterate on links
	for elem in links:
		# check if name attribute is "left_pad" or "right_pad"
		if train_model.sdf_check_name_attribute(elem, "left_pad"):
			# get y coordinates for left pad
			pad_pose_value = find_pad_position("left", left_pad_width)
				
			# get "pose" element from sdf and modify its value
			pad_pose = train_model.sdf_get_pose(elem)
			for pose in pad_pose:
				train_model.sdf_set_inner_value(pose, pad_pose_value)
			
			# get "length" element from sdf and modify its value
			pad_width = train_model.sdf_get_height(elem)
			for width in pad_width:
				train_model.sdf_set_inner_value(width, left_pad_width)

			# modify "train_structure" dictionary
			train_model.train_struct_set_width("pads", "left", left_pad_width)
			train_model.train_struct_set_left_pose("pads", pad_pose_value)
			
		elif train_model.sdf_check_name_attribute(elem, "right_pad"):
			pad_pose_value = find_pad_position("right", right_pad_width)
				
			pad_pose = train_model.sdf_get_pose(elem)
			for pose in pad_pose:
				train_model.sdf_set_inner_value(pose, pad_pose_value)
			
			pad_width = train_model.sdf_get_height(elem)
			for width in pad_width:
				train_model.sdf_set_inner_value(width, right_pad_width)

			train_model.train_struct_set_width("pads", "right", right_pad_width)
			train_model.train_struct_set_right_pose("pads", pad_pose_value)

# creation of message for init inspector		
def create_message(command):
	message = { 
		'cmd': command,
		'train_description':{
			'axis':{
				'position': train_model.train_struct_get_coordinates("axis"),
				'lenght': train_model.train_struct_get_width("axis"),
				'pads':{
					'num': 2,
					'coordinates':[
						train_model.train_struct_get_coordinates("pads")[0],
						train_model.train_struct_get_coordinates("pads")[1]
					],
					'disk':{
						'radius': train_model.train_struct_get_radius("disks"),
					},
				}
			}
		}
	}
	return message

def create_train_request(modelname, px, py, pz, lpw, rpw):
	"""Create a SpawnModelRequest with the parameters of the train given.
	modelname: name of the model for gazebo
	px py pz: position of the train (and it's collision train)
	lpw: left pad width
	rpw: right pad width"""
	
	set_pads_width(lpw,rpw)

	train = deepcopy(train_model.get_sdf())

	req = SpawnModelRequest()
	req.model_name = modelname
	req.model_xml = train
	req.initial_pose.position.x = px
	req.initial_pose.position.y = py
	req.initial_pose.position.z = pz

	return req

def spawn_notify(pub):
	if not rospy.is_shutdown():
		msg = create_message("check")
		json_msg = json.dumps(msg)
		print json_msg
		pub.publish(json_msg)

if __name__ == '__main__':

	# connection to spawn_sdf_model service and to delete_model service of gazebo
	rospy.init_node('spawn_models', anonymous=True)
	delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
	rospy.loginfo("Waiting for /gazebo/delete_model service...")
	delete_srv.wait_for_service()
	rospy.loginfo("Connected to delete service!")

	spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
	spawn_srv.wait_for_service()
	rospy.loginfo("Connected to spawn service!")

	pub = rospy.Publisher('spawn_channel', String, queue_size=10)
	
	out0 = "Delete the train model"
	out1 = "Spawn a train model with 'good' pads"
	out2 = "Spawn a train model with 'bad' pads"
	out3 = "Spawn a train model with 'random' pads"
	out4 = "EXIT"

	i = 0

	while True:
		type=input("What do you want to do? \n\t0\t>>"+out0+"\n\t1\t>>"+out1+"\n\t2\t>>"+out2+"\n\t3\t>>"+out3+"\n\tother\t>>"+out4+"\n")
		if type == 1:
			i+=1
			# Spawn train with good pads
			print "Spawning train with good pads"
			req = create_train_request("train_model_"+str(i), 		# model name
										0.0, 0.0, 0.0,		# train initial position
										good_pad_width, good_pad_width)  		# pads length (status)
			spawn_srv.call(req)
		elif type == 2:
			i+=1
			print "Spawning train with bad pads"
			req = create_train_request("train_model_"+str(i), 
										0.0, 0.0, 0.0,
										bad_pad_width, bad_pad_width) 
			spawn_srv.call(req)
		elif type == 3:
			i+=1
			print "Spawning train with random pads width"
			lpw = random.uniform(bad_pad_width, good_pad_width)
			print "-left pad width "+str(lpw)
			rpw = random.uniform(bad_pad_width, good_pad_width)
			print "-right pad width "+str(rpw)
			req = create_train_request("train_model_"+str(i), 
										0.0, 0.0, 0.0,
										lpw, rpw) 
			spawn_srv.call(req)
		elif type == 0:
			print "Deleting train model"
			delete_srv("train_model_"+str(i))
			continue
		else:	
			break
		
		# notify the spawn on the channel
		try:
			spawn_notify(pub)
		except rospy.ROSInterruptException:
			pass
