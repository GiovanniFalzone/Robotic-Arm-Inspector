#!/usr/bin/python
import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

class robot_motion_lib():
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('arm_inspector_move_node',	anonymous=True)

		## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
		## the robot:
		robot = moveit_commander.RobotCommander()

		## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
		## to the world surrounding the robot:
		scene = moveit_commander.PlanningSceneInterface()

		## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
		## to one group of joints.  In this case the group is the joints in the Panda
		## arm so we set ``group_name = panda_arm``. If you are using a different robot,
		## you should change this value to the name of your robot arm planning group.
		## This interface can be used to plan and execute motions on the Panda:
		group_name = "manipulator"
		group = moveit_commander.MoveGroupCommander(group_name)

		## We create a `DisplayTrajectory`_ publisher which is used later to publish
		## trajectories for RViz to visualize:
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
														moveit_msgs.msg.DisplayTrajectory,
														queue_size=20)
		planning_frame = group.get_planning_frame()
		eef_link = group.get_end_effector_link()
		group_names = robot.get_group_names()

		print("------------------------------\n")
		print "Reference frame: %s" % planning_frame
		print "End effector: %s" % eef_link
		print "Robot Groups:", robot.get_group_names()
		# print "Printing robot state"
		# print robot.get_current_state()
		print("------------------------------\n")
		self.robot = robot
		self.scene = scene
		self.group = group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names
		self.tolerance = 0.01

	def check_tolerance(self, goal, actual):
		all_equal = True
		if type(goal) is list:
			for index in range(len(goal)):
				if abs(actual[index] - goal[index]) > self.tolerance:
					return False

		elif type(goal) is geometry_msgs.msg.PoseStamped:
			return self.check_tolerance(goal.pose, actual.pose)

		elif type(goal) is geometry_msgs.msg.Pose:
			return self.check_tolerance(pose_to_list(goal), pose_to_list(actual))

		return True

	def set_orientation_by_rpy(self, pos, roll=math.pi/2, pitch=0, yaw=0):
		q = quaternion_from_euler(roll, pitch, yaw)
		pos.orientation.w = q[0]
		pos.orientation.x = q[1]
		pos.orientation.y = q[2]
		pos.orientation.z = q[3]
		return pos

	def get_pos_obj(self, vect):
		pos = geometry_msgs.msg.Pose()
		# get quaternion from rpy
		pos = self.set_orientation_by_rpy(pos, vect[2], vect[3], vect[4])
		pos.position.x = vect[0]
		pos.position.y = vect[1]
		pos.position.z = vect[2]
		return pos

	def move_in_pos(self, pos):
		print(pos)
		self.group.set_pose_target(pos)
		plan = self.group.go(wait=True)
		self.group.stop() # to ensures that there is no residual movement
		self.group.clear_pose_targets()
		current_pos = self.group.get_current_pose().pose
		return self.check_tolerance(pos, current_pos)

	def move_in_xyz_rpy(self, vect):
		pos_goal = self.get_pos_obj(vect)
		self.move_in_pos(pos_goal)

	def set_joint(self, joints_vect):
		self.group.go(joints_vect, wait=True)
		self.group.stop()
		current_joints = self.group.get_current_joint_values()
		return self.check_tolerance(joints_vect, current_joints)

	def move_in_points(self, points):
		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
		(plan, fraction) = self.group.compute_cartesian_path(
											points,   # waypoints to follow
											0.01,        # eef_step
											0.0)         # jump_threshold
		self.group.execute(plan, wait=True)

	def follow_points(self, points):
		for pos in points:
			self.move_in_pos(pos)

	def follow_line(self, start_pos, step_x, step_y, step_z, max_step,  roll=math.pi/2, pitch=0, yaw=0):
		x = start_pos[0]
		y = start_pos[1]
		z = start_pos[2]
		points = []
		pos = geometry_msgs.msg.Pose()
		for i in range(0, max_step):
			pos.position.x = x + i*step_x
			pos.position.y = y + i*step_y
			pos.position.z = z + i*step_z
			pos = self.set_orientation_by_rpy(pos, roll, pitch, yaw)
			points.append(copy.deepcopy(pos))
		self.follow_points(points)

	def follow_cone_base_y(self, vertex_pos, radius=0.1, dist=0.1, angle=2*math.pi, clockwise=0, num_points=10):
		x = vertex_pos[0]
		y = vertex_pos[1] - dist
		z = vertex_pos[2]
		roll = math.pi/2
		pitch = 0
		yaw = 0
		self.move_in_xyz_rpy([x, y, z, roll, pitch, yaw])
		direction = 1 - 2*clockwise
		deg_step = angle/num_points
		points = []
		pos = geometry_msgs.msg.Pose()
		for i in range(0, num_points):
			actual_deg = i*deg_step*direction
			pos.position.x = x + radius*math.cos(actual_deg)
			pos.position.y = y
			pos.position.z = z + radius*math.sin(actual_deg)
			if(dist>0):
				roll = math.pi/2 - math.atan(dist/radius)*math.cos(actual_deg)
				pitch = 0 - math.atan(dist/radius)*math.sin(actual_deg)
				yaw = 0
			else:
				roll = math.pi/2
				pitch = 0
				yaw = 0
			pos = self.set_orientation_by_rpy(pos, roll, pitch, yaw)
			points.append(copy.deepcopy(pos))
		self.follow_points(points)

	def follow_circle_y(self, center_pos, radius=0.1, angle=2*math.pi, clockwise=0, num_points=10):
		self.follow_cone_base_y(center_pos, radius, 0, angle, clockwise, num_points)

	def follow_cone_base_x(self, vertex_pos, radius=0.1, dist=0.1, angle=2*math.pi, clockwise=0, num_points=10):
		x = vertex_pos[0] - dist
		y = vertex_pos[1]
		z = vertex_pos[2]
		roll = 0
		pitch = 0
		yaw = 0
		self.move_in_xyz_rpy([x, y, z, roll, pitch, yaw])
		direction = 1 - 2*clockwise
		deg_step = angle/num_points
		points = []
		pos = geometry_msgs.msg.Pose()
		for i in range(0, num_points):
			actual_deg = i*deg_step*direction
			pos.position.x = x
			pos.position.y = y + radius*math.cos(actual_deg)
			pos.position.z = z + radius*math.sin(actual_deg)
			if(dist>0):
				roll = 0 - math.atan(dist/radius)*math.cos(actual_deg)
				pitch = 0 - math.atan(dist/radius)*math.sin(actual_deg)
				yaw = 0
			else:
				roll = 0
				pitch = 0
				yaw = 0
			pos = self.set_orientation_by_rpy(pos, roll, pitch, yaw)
			points.append(copy.deepcopy(pos))
		self.follow_points(points)

	def follow_circle_x(self, center_pos, radius=0.1, angle=2*math.pi, clockwise=0, num_points=10):
		self.follow_cone_base_x(center_pos, radius, 0, angle, clockwise, num_points)
