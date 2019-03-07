#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler

from xml.dom import minidom

MODELS_DIR = "/home/rob/catkin_ws/src/robotic_arm_inspector/models/"
sdf_train = minidom.parse(MODELS_DIR + "train_model/model.sdf")

good_pad = 0.2		#length of pad in good status
bad_pad = 0.05
lpad_offset = 1.23
rpad_offset = 2.76

def set_pad_status(pl):
	links = sdf_train.getElementsByTagName("link")
	for el in links:
		if el.getAttribute('name') == "left_pad":		# find left pad
			pad_poses = el.getElementsByTagName("pose")	# pose is based on pad length (status)
			pad_y = lpad_offset - pl/2
			pad_poses[0].firstChild.nodeValue = "0.0 "+ str(pad_y) +" 1.5 0.0 1.5707963267948966 1.5707963267948966"

			pad_length = el.getElementsByTagName("length")
			for pad_l in pad_length:
				pad_l.firstChild.nodeValue = pl

		elif  el.getAttribute('name') == "right_pad":
			pad_poses = el.getElementsByTagName("pose")	# pose is based on pad length (status)
			pad_y = rpad_offset + pl/2
			pad_poses[0].firstChild.nodeValue = "0.0 "+ str(pad_y) +" 1.5 0.0 1.5707963267948966 1.5707963267948966"

			pad_length = el.getElementsByTagName("length")
			for pad_l in pad_length:
				pad_l.firstChild.nodeValue = pl
		
			

def create_train_request(modelname, px, py, pz, rr, rp, ry, pl):
	"""Create a SpawnModelRequest with the parameters of the train given.
	modelname: name of the model for gazebo
	px py pz: position of the cube (and it's collision cube)
	rr rp ry: rotation (roll, pitch, yaw) of the model"""
	set_pad_status(pl)

	train = deepcopy(sdf_train.documentElement.toxml())

	req = SpawnModelRequest()
	req.model_name = modelname
	req.model_xml = train
	# req.initial_pose.position.x = px
	# req.initial_pose.position.y = py
	req.initial_pose.position.z = pz

	# q = quaternion_from_euler(rr, rp, ry)
	# req.initial_pose.orientation.x = q[0]
	# req.initial_pose.orientation.y = q[1]
	# req.initial_pose.orientation.z = q[2]
	# req.initial_pose.orientation.w = q[3]

	return req


if __name__ == '__main__':
	rospy.init_node('spawn_models')
	spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
	spawn_srv.wait_for_service()
	rospy.loginfo("Connected to service!")
	
	while 1:
		type=input("What type of train do you want to spawn? \n\t1 -> with good pads\n\t2 -> with bad pads\n\t0 -> EXIT\n")
		if type == 1:
			# Spawn train with good pads
			rospy.loginfo("Spawning train with good pads")
			req = create_train_request("train_good", 		# model name
										0.0, 0.0, 0.8,		# train initial position
										0.0, 0.0, 0.0,		# train initial rotation
										good_pad)  		# pads length (status)
			spawn_srv.call(req)
		elif type == 2:
			rospy.loginfo("Spawning train with bad pads")
			req = create_train_request("train_bad", 
										0.0, 0.0, 0.8,
										0.0, 0.0, 0.0,
										bad_pad) 
			spawn_srv.call(req)
		else:	
			break