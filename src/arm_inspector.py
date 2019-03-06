#!/usr/bin/python
import sys
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

	def move_in_pos(self, x, y, z, roll, pitch, yaw):
		pos_goal = geometry_msgs.msg.Pose()

		# get quaternion from rpy
		q = quaternion_from_euler(roll, pitch, yaw)
		pos_goal.orientation.w = q[0]
		pos_goal.orientation.x = q[1]
		pos_goal.orientation.y = q[2]
		pos_goal.orientation.z = q[3]
		pos_goal.position.x = x
		pos_goal.position.y = y
		pos_goal.position.z = z
		self.group.set_pose_target(pos_goal)

		plan = self.group.go(wait=True)
		self.group.stop() # to ensures that there is no residual movement
		self.group.clear_pose_targets()
		current_pos = self.group.get_current_pose().pose
		return self.check_tolerance(pos_goal, current_pos)

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
		(plan, fraction) = group.compute_cartesian_path(
											points,   # waypoints to follow
											0.01,        # eef_step
											0.0)         # jump_threshold
		self.group.execute(plan, wait=True)

	def inspect_point(x, y, z):
		radius = 0.2
		num_points = 10
		deg_step = 2*pi/num_points
		points = []
		pos = geometry_msgs.msg.Pose()
		for i in range(0, num_points):
			pos.position.y = y + radius*math.cos(i*deg_step)
			pos.position.z = z + radius*math.sin(i*deg_step)
			points.append(copy.deepcopy(pos))
		move_in_points(points)


	def exec_plan(self, plan):
		self.group.execute(plan, wait=True)

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
