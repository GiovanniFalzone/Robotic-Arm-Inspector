#!/usr/bin/python
import rospy
from math import pi
from arm_inspector import *
from std_msgs.msg import String

inspector = arm_inspector()
    
def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	cmd = data.data
	# msg = 'Press \n\t-> 1 to move in initial position \n\t-> 2 to move in max estension position \n\t-> 3 to move in checking position \n\t-> 4 to inspect circle\n\t-> 5 to inspect point\n'
	# cmd = raw_input(msg)
	if('1' in cmd):
		inspector.move_in_sleep_position()
	elif('2' in cmd):
		inspector.move_in_waiting_position()
	elif('3' in cmd):
		inspector.move_in_checking_position()
	elif('4' in cmd):
		inspector.inspect_circle(0, 0.25, 1.7, 0.1, 5)
	elif('5' in cmd):
		inspector.inspect_point(0, 0.5, 1.7, 0.2, 0.2, 5)
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
