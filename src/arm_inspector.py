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

class arm_inspector():
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

	def get_pos_obj(self, x, y, z, roll, pitch, yaw):
		pos = geometry_msgs.msg.Pose()
		# get quaternion from rpy
		pos = self.set_orientation_by_rpy(pos, roll, pitch, yaw)
		pos.position.x = x
		pos.position.y = y
		pos.position.z = z
		return pos

	def move_in_pos(self, pos):
		print(pos)
		self.group.set_pose_target(pos)
		plan = self.group.go(wait=True)
		self.group.stop() # to ensures that there is no residual movement
		self.group.clear_pose_targets()
		current_pos = self.group.get_current_pose().pose
		return self.check_tolerance(pos, current_pos)

	def move_in_xyz_rpy(self, x, y, z, roll, pitch, yaw):
		pos_goal = self.get_pos_obj(x, y, z, roll, pitch, yaw)
		self.move_in_pos(pos_goal)

	def set_joint(self, a0, a1, a2, a3, a4, a5):
		joint_goal = self.group.get_current_joint_values()
		joint_goal[0] = a0
		joint_goal[1] = a1
		joint_goal[2] = a2
		joint_goal[3] = a3
		joint_goal[4] = a4
		joint_goal[5] = a5
		self.group.go(joint_goal, wait=True)
		self.group.stop()
		current_joints = self.group.get_current_joint_values()
		return self.check_tolerance(joint_goal, current_joints)

	def move_in_points(self, points):
		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
		(plan, fraction) = self.group.compute_cartesian_path(
											points,   # waypoints to follow
											0.01,        # eef_step
											0.0)         # jump_threshold
		self.group.execute(plan, wait=True)

	def move_in_sleep_position(self):
		self.set_joint(0, 0, 0, 0, 0, 0)

	def move_in_waiting_position(self):

		self.set_joint(0, -math.pi/4, 0, -math.pi/2, 0, -math.pi/4)

	def move_in_checking_position(self):
		self.move_in_xyz_rpy(0, 0.25, 1.9, math.pi/2, 0, 0)

	def follow_points(self, points):
		for pos in points:
			self.move_in_pos(pos)

	def inspect_point(self, x, y, z, radius=0.1, dist=0.1, num_points=10):
		self.move_in_waiting_position()
		y = y - dist
		roll = math.pi/2
		pitch = 0
		yaw = 0
		self.move_in_xyz_rpy(x, y, z, roll, pitch, yaw)
		deg_step = 2*math.pi/num_points
		points = []
		pos = geometry_msgs.msg.Pose()
		for i in range(0, num_points):
			pos.position.x = x + radius*math.cos(i*deg_step)
			pos.position.y = y
			pos.position.z = z + radius*math.sin(i*deg_step)
			if(dist>0):
				roll = math.pi/2 - math.atan(dist/radius)*math.cos(i*deg_step)
				pitch = 0 - math.atan(dist/radius)*math.sin(i*deg_step)
				yaw = 0
			else:
				roll = math.pi/2
				pitch = 0
				yaw = 0
			pos = self.set_orientation_by_rpy(pos, roll, pitch, yaw)
			points.append(copy.deepcopy(pos))
		self.follow_points(points)
		roll = math.pi/2
		pitch = 0
		yaw = 0
		self.move_in_waiting_position()

# this method get a center position and radius of a circle and move to N points on the circle
	def inspect_circle(self, x, y, z, radius=0.1, num_points=10):
		self.inspect_point(x, y, z, radius, 0, num_points)