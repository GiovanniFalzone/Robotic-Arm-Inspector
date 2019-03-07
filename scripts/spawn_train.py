#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler

from xml.dom import minidom

MODELS_DIR = "/home/rob/catkin_ws/src/robotic_arm_inspector/models/"
sdf_train = minidom.parse(MODELS_DIR + "train_model/model.sdf")


def create_train_request(modelname, px, py, pz, rr, rp, ry):
	"""Create a SpawnModelRequest with the parameters of the train given.
	modelname: name of the model for gazebo
	px py pz: position of the cube (and it's collision cube)
	rr rp ry: rotation (roll, pitch, yaw) of the model"""
	train = deepcopy(sdf_train.documentElement.toxml())

	req = SpawnModelRequest()
	req.model_name = modelname
	req.model_xml = train
	req.initial_pose.position.x = px
	req.initial_pose.position.y = py
	req.initial_pose.position.z = pz

	q = quaternion_from_euler(rr, rp, ry)
	req.initial_pose.orientation.x = q[0]
	req.initial_pose.orientation.y = q[1]
	req.initial_pose.orientation.z = q[2]
	req.initial_pose.orientation.w = q[3]

	return req


if __name__ == '__main__':
	rospy.init_node('spawn_models')
	spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
	spawn_srv.wait_for_service()
	rospy.loginfo("Connected to service!")
	
	while 1:
		type=input("What type of train do you want to spawn? \n\t1 -> with good pads\n\t2 -> others\n\t0 -> EXIT")
		if type == 1:
			# Spawn train with good pads
			rospy.loginfo("Spawning train with good pads")
			req = create_train_request("train_good",
										0.0, 0.0, 0.51,  # position
										0.0, 0.0, 0.0)  # rotation
			spawn_srv.call(req)
		elif type == 0:
			break
		else:	
			rospy.loginfo("Ciao")