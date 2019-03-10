#!/usr/bin/python
from math import pi
from train_inspector import *

motion_lib = robot_motion_lib()
inspector = train_inspector()

def main():
	msg = 'Press \n\
	\t-> 1 to move in initial position \n\
	\t-> 2 to move in max estension position \n\
	\t-> 3 to move in checking position \n\
	\t-> 4 follow circle y\n\
	\t-> 5 follow cone y\n\
	\t-> 6 follow line x\n\
	\t-> 7 follow circle x\n\
	\t-> 8 follow cone\n\
	\t-> axis to check axis\n\
	\t-> pad to check pads\n'
	cmd = raw_input(msg)
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
		motion_lib.follow_line([0.5, 1, 1], 0, 0.2, 0, 5)
	elif('7' in cmd):
		motion_lib.follow_circle_x([-0.5, 2, 2], 0.1, math.pi, 1, 5)
	elif('8' in cmd):
		motion_lib.follow_cone_base_x([-0.5, 2, 2], 0.2, 0.2, math.pi, 1, 5)
	elif('axis' in cmd):
		inspector.inspect_axis()
	elif('pad' in cmd):
		inspector.inspect_pads()
	elif('pos' in cmd):
		cmd = raw_input('insert cordinates:[x,y,z]')
		coords = cmd.split(']')[0].split('[')[1].split(',')
		x = eval(coords[0])
		y = eval(coords[1])
		z = eval(coords[2])
		print('moving to: ' + str([x,y,z]))
		motion_lib.move_in_xyz_rpy([x,y,z,0,0,0])

	else:
		print('Wrong input \n')

if __name__ == '__main__':
	while(True):
		try:
			main()
		except rospy.ROSInterruptException:
			print('rospy exception')
			break
		except KeyboardInterrupt:
			print('keyboard interrupt')
			break
