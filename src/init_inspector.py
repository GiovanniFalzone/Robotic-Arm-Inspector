#!/usr/bin/python
import rospy
from math import pi
from train_inspector import *
from std_msgs.msg import String
import ast

motion_lib = robot_motion_lib()
inspector = train_inspector()
    
def callback(data):
	msg = ast.literal_eval(data.data)
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)
	cmd =  msg.get("cmd")
	train_desc = msg.get("train_description")

	# msg = 'Press \n\t-> 1 to move in initial position \n\t-> 2 to move in max estension position \n\t-> 3 to move in checking position \n\t-> 4 to inspect circle\n\t-> 5 to inspect point\n'
	# cmd = raw_input(msg)
	if('1' in cmd):
		inspector.move_in_sleep_position()
	elif('2' in cmd):
		inspector.move_in_waiting_position()
	elif('3' in cmd):
		inspector.move_in_checking_position()
	elif('4' in cmd):
		motion_lib.follow_circle_y([0, 2, 1.7], 0.1, math.pi, 1, 5)
	elif('5' in cmd):
		motion_lib.follow_cone_base_y([0, 2, 1.7], 0.2, 0.2, math.pi, 1, 5)
	elif('6' in cmd):
		motion_lib.follow_line([0, 1.5, 1.5], 0, 0.02, 0, 10)
	elif('7' in cmd):
		motion_lib.follow_circle_x([0, 2, 1.7], 0.1, math.pi, 1, 5)
	elif('8' in cmd):
		motion_lib.follow_cone_base_x([0, 2, 1.7], 0.2, 0.2, math.pi, 1, 5)
	elif('axis' in cmd):
		inspector.inspect_axis()
	elif('pad' in cmd):
		inspector.inspect_pads()
	elif('check' in cmd):
		inspector.check_train(train_desc)

	else:
		print('Wrong input \n')

if __name__ == '__main__':	
	
	rospy.Subscriber("spawn_channel", String, callback)
	print('Waiting for message on spawn channel\n')
	rospy.spin()
	# while(True):
	# 	try:
	# 		main()
	# 	except rospy.ROSInterruptException:
	# 		print('rospy exception')
	# 		break
	# 	except KeyboardInterrupt:
	# 		print('keyboard interrupt')
	# 		break
