#!/usr/bin/python
from math import pi
from arm_inspector import *

inspector = arm_inspector()




def main():
		msg = 'Press \n\t-> 1 to move in initial position \n\t-> 2 to move in max estension position \n\t-> 3 to move in checking position \n\t-> 4 to inspect point\n'
		cmd = raw_input(msg)
		if('1' in cmd):
			inspector.set_joint(0, 0, 0, 0, 0, 0)
		elif('2' in cmd):
			inspector.set_joint(0, -pi/4, 0, -pi/2, 0, pi/3)
		elif('3' in cmd):
			inspector.move_in_pos(0, 0.25, 0.9, pi/2, 0, pi/2)
		elif('4' in cmd):
			inspect_point(0, 0.5, 0.5)
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
