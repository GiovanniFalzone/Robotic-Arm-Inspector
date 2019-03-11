#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse	# spawn messages to communicate with gazebo
from std_msgs.msg import String													# to exchange string messages
from copy import deepcopy
#from tf.transformations import quaternion_from_euler
from xml.dom import minidom														# to parse sdf model
import json
from os.path import expanduser

from train_model_parser import *

train_model = train_model()

# global variables
# a pad is considered "good" or "bad" w.r.t. its width
good_pad_width = 0.2		
bad_pad_width = 0.05


message = { 
	'cmd':'check',
	'train_description':{
		'axis':{
			'position':[0, 1.5, 0],
			'lenght':3.0,
			'pads':{
				'num':2,
				'coordinates':[
					[0, 0, 0],
					[0, 0, 0]
				],
				'disk':{
					'radius': 0.5,
				},
			}
		}
	}
}

# MODELS_DIR = "/home/rob/catkin_ws/src/robotic_arm_inspector/models/"
# sdf_train = minidom.parse(MODELS_DIR + "train_model/model.sdf")

def set_pad_status(pw):		# pads are little if they are in bad status

	train_model.set_pads_width(pw, pw)
	
	left_pad_pos = train_model.get_pad_position("left", pw)
	right_pad_pos = train_model.get_pad_position("right", pw)
	message["train_description"]["axis"]["pads"]["coordinates"] = [left_pad_pos, right_pad_pos]

	# links = train_model.get_links()
	# lpad_y = pl/2
	# rpad_y = pl/2
	# message["train_description"]["axis"]["pads"]["coordinates"] = [[0.0, lpad_y, 2.0], [0.0, rpad_y, 2.0]]
	# for el in links:
	# 	if train_model.check_attribute(el, "name", "left_pad"):		# find left pad

	# 		#pad_poses = el.getElementsByTagName("pose")
	# 		pad_pose = train_model.get_pose(el)
	# 		pad_pose_value = "0.0 " + str(lpad_y) + " 2.0 " + rotation_string + ""
	# 		train_model.set_value(pad_pose[0], pad_pose_value)
	# 		#pad_poses[0].firstChild.nodeValue = "0.0 "+ str(lpad_y) +" 2.0 0.0 1.5707963267948966 1.5707963267948966"
	# 		pad_width = train_model.get_length(el)

	# 		for pad_l in pad_width:
	# 			train_model.set_value(pad_l, pl)

	# 	elif  el.getAttribute('name') == "right_pad":
	# 		pad_poses = el.getElementsByTagName("pose")	# pose is based on pad length (status)
	# 		pad_poses[0].firstChild.nodeValue = "0.0 "+ str(rpad_y) +" 2.0 0.0 1.5707963267948966 1.5707963267948966"

	# 		pad_length = el.getElementsByTagName("length")
	# 		for pad_l in pad_length:
	# 			pad_l.firstChild.nodeValue = pl

def create_train_request(modelname, px, py, pz, pw):
	"""Create a SpawnModelRequest with the parameters of the train given.
	modelname: name of the model for gazebo
	px py pz: position of the cube (and it's collision cube)
	rr rp ry: rotation (roll, pitch, yaw) of the model"""
	set_pad_status(pw)

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
		message["cmd"] = "check"
		json_msg = json.dumps(message)
		print json_msg
		pub.publish(json_msg)

if __name__ == '__main__':

	# connection to spawn_sdf_model service of gazebo
	rospy.init_node('spawn_models', anonymous=True)
	spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
	spawn_srv.wait_for_service()
	rospy.loginfo("Connected to service!")

	pub = rospy.Publisher('spawn_channel', String, queue_size=10)
	
	while True:
		type=input("What type of train do you want to spawn? \n\t1 -> with good pads\n\t2 -> with bad pads\n\t0 -> EXIT\n")
		if type == 1:
			# Spawn train with good pads
			rospy.loginfo("Spawning train with good pads")
			req = create_train_request("train_good", 		# model name
										0.0, 0.0, 0.0,		# train initial position
										good_pad_width)  		# pads length (status)
			spawn_srv.call(req)
		elif type == 2:
			rospy.loginfo("Spawning train with bad pads")
			req = create_train_request("train_bad", 
										0.0, 0.0, 0.0,
										bad_pad_width) 
			spawn_srv.call(req)
		else:	
			break
		
		# notify the spawn on the channel
		try:
			spawn_notify(pub)
		except rospy.ROSInterruptException:
			pass
