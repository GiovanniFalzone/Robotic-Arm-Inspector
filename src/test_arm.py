#!/usr/bin/python
from math import pi
from train_inspector import *

motion_lib = robot_motion_lib()
inspector = train_inspector()


train_description = {
	'axis':{
		'position':[0, 1.5, 2],
		'lenght':2,
		'pads':{
			'num':2,
			'coordinates': [[0.0, 1.2, 2.0], [0.0, 1.8, 2.0]],
			'disk':{
				'radius': 0.25,
			},
		}
	}
}

def get_number(msg):
	text = raw_input(msg)
	return eval(text)

def get_vector(msg):
	text = raw_input(msg + ' ([x,y,z]):')
	coords = text.split(']')[0].split('[')[1].split(',')
	x = eval(coords[0])
	y = eval(coords[1])
	z = eval(coords[2])
	return [x, y, z]

def get_position():
	return get_vector('Insert coordinates')

def main():
	msg = 'Press \n\
	\t-> 1 to move in initial position \n\
	\t-> 2 to move in max estension position \n\
	\t-> 3 to move in checking position \n\
	\t-> line to move following a cone base \n\
	\t-> circle to move following a circle \n\
	\t-> cone to move in checking position \n\
	\t-> axis to check axis\n\
	\t-> pad to check pads\n\
	\t-> check to check the train\n'
	cmd = raw_input(msg)
	if('1' in cmd):
		inspector.move_in_sleep_position()
	elif('2' in cmd):
		inspector.move_in_waiting_position()
	elif('3' in cmd):
		inspector.move_in_checking_position()
	elif('circle' in cmd):
		axis = raw_input('which axis?(x, y or z)') 
		vect_pos = get_position()
		print('Circle center: ' + str(vect_pos))
		if('x' in axis):
			motion_lib.follow_circle_x(vect_pos)
		elif('y' in axis):
			motion_lib.follow_circle_y(vect_pos)
		elif('z' in axis):
			motion_lib.follow_circle_z(vect_pos)

	elif('cone' in cmd):
		axis = raw_input('which axis?(x, y or z)') 
		vect_pos = get_position()
		print('Cone vertex: ' + str(vect_pos))
		if('x' in axis):
			motion_lib.follow_cone_base_x(vect_pos)
		elif('y' in axis):
			motion_lib.follow_cone_base_y(vect_pos)
		elif('z' in axis):
			motion_lib.follow_cone_base_z(vect_pos)

	elif('line' in cmd):
		vect_pos = get_position()
		steps = get_number('Insert How many step:')
		direction = get_vector('Insert Direction (Delta for each axis)')
		motion_lib.follow_line(vect_pos, direction, steps)
	elif('train_axis' in cmd):
		inspector.inspect_axis()
	elif('traind_pads' in cmd):
		inspector.inspect_pads()
	elif('pos' in cmd):
		vect_pos = get_position()
		vect_pos.extend([0,0,0]),
		print('moving to: ' + str(vect_pos))
		motion_lib.move_in_xyz_rpy(vect_pos)

	elif('rotate' in cmd):
		vect = get_vector('Insert rpy as: ')
		motion_lib.rotate(vect)

	elif('pad' in cmd):
		inspector.inspect_pads()

	elif('axis' in cmd):
		inspector.inspect_axis()

	elif('check' in cmd):
		inspector.check_train(train_description)

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
