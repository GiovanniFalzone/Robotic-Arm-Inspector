from xml.dom import minidom
from math import pi
from os.path import expanduser
from pprint import pprint

home = expanduser("~")
MODELS_DIR = home+"/catkin_ws/src/robotic_arm_inspector/models/"
rotation_string = "0.0 " + str(pi/2) + " " + str(pi/2) +" "

class train_model():
	def __init__(self):
		self.sdf_train = minidom.parse(MODELS_DIR + "train_model/model.sdf")

		# train_structure is a dictionary containing train description subdivided in axis, wheels, disks and pads
		self.train_structure = self.init_dictionaries()

	# sdf_file management
	def get_sdf(self):
		"""Return the sdf structure of train model"""
		return self.sdf_train.documentElement.toxml()

	# returns a DOM node for the subelement with tag name "tagname"
	def sdf_get_subelements(self, element, tagname):
		"""Returns the DOM subnode or subnodes specified by "tagname" """
		return element.getElementsByTagName(tagname)

	def sdf_get_links(self):
		"""Returns the array of "link" node of the sdf model"""
		return self.sdf_get_subelements(self.sdf_train, "link")

	def sdf_get_pose(self, element):
		"""Returns the "pose" tag for element """
		return self.sdf_get_subelements(element, "pose")
	
	def sdf_get_length(self, element):
		"""Returns the "length" tag for element """
		return self.sdf_get_subelements(element, "length")

	def sdf_get_height(self, element):
		"""Returns the "height" tag for element """
		return self.sdf_get_subelements(element, "height")

	def sdf_get_radius(self, element):
		"""Returns the "radius" tag for element """
		return self.sdf_get_subelements(element, "radius")

	# returns the value of the attribute "attr" for the element "element"
	def sdf_get_attribute(self, element, attr):
		"""Returns the value of the attribute "attr" for element """
		return element.getAttribute(attr)

	# returns the value incapsulated by the given tag
	def sdf_get_inner_value(self, element):
		"""Returns the text value for the element """
		return element[0].firstChild.nodeValue

	def sdf_set_inner_value(self, element, value):
		"""Set the text value of the element with "value" """
		element.firstChild.nodeValue = value

	def sdf_set_pad_color(self, element, color):
		"""Set pad material with the specified Gazebo color """
		material_name = self.sdf_get_subelements(element, "name")
		self.sdf_set_inner_value(material_name[0], "Gazebo/"+color)

	def	sdf_check_attribute(self, element, attr, value):
		"""Checks if the attribute "attr" has the specified "value" """
		return self.sdf_get_attribute(element, attr) == value

	def sdf_check_name_attribute(self, element, value):
		"""Checks if the attribute "name" has the specified "value" """
		return self.sdf_check_attribute(element, "name", value)

	# train dictionary management

	# create a dictionary of the type:
	#	{ "name" : "train", "axis" : {"pose": .., "width": .., "radius": ..}, "wheels":{....}...}
	def init_dictionaries(self):
		train_dict = { 
			'name' : 'train_model_name',
			'axis' : {
				'pose' : 'x y z roll pitch yaw',
				'coordinates' : [0.0, 0.0, 0.1],
				'radius' : 'radius',
				'width': 'width',
			},
			'wheels' : {
				'pose' : { 'left': 'lx ly lz lroll lpitch lyaw', 'right' : 'rx ry rz rroll rpitch ryaw'},
				'coordinates' : {'left' : [0.0, 0.0, 0.0], 'right': [0.0, 0.0, 0.0]},
				'radius' : 'wheels radius',
				'width': 'wheels width',
			},
			'disks' : {
				'pose' : { 'left': 'lx ly lz lroll lpitch lyaw', 'right' : 'rx ry rz rroll rpitch ryaw'},
				'coordinates' : {'left' : [0.0, 0.0, 0.0], 'right': [0.0, 0.0, 0.0]},
				'radius' : 'disks radius',
				'width': 'disks width',
			},
			'pads' : {
				'pose' : { 'left': 'lx ly lz lroll lpitch lyaw', 'right' : 'rx ry rz rroll rpitch ryaw'},
				'coordinates' : {'left' : [0.0, 0.0, 0.0], 'right': [0.0, 0.0, 0.0]},
				'width': {'left': 'left width', 'right': 'right_width'},
			}
		}

		links = self.sdf_get_links()

		for elem in links:
			if self.sdf_check_name_attribute(elem, "axis"):
				train_dict.get('axis')['width'] = float(self.sdf_get_inner_value(self.sdf_get_length(elem)))
				train_dict.get('axis')['radius'] = float(self.sdf_get_inner_value(self.sdf_get_radius(elem)))
				axis_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
				train_dict.get('axis')['pose'] = axis_pose
				train_dict.get('axis')['coordinates'] = self.pose_to_xyz(axis_pose)
			elif self.sdf_check_name_attribute(elem, 'left_wheel'):
				train_dict.get('wheels')['width'] = float(self.sdf_get_inner_value(self.sdf_get_length(elem)))
				train_dict.get('wheels')['radius'] = float(self.sdf_get_inner_value(self.sdf_get_radius(elem)))
				left_wheel_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
				train_dict.get('wheels')['pose']['left'] = left_wheel_pose
				train_dict.get('wheels')['coordinates']['left'] = self.pose_to_xyz(left_wheel_pose)
			elif self.sdf_check_name_attribute(elem, 'right_wheel'):
				right_wheel_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
				train_dict.get('wheels')['pose']['right'] = right_wheel_pose
				train_dict.get('wheels')['coordinates']['right'] = self.pose_to_xyz(right_wheel_pose)
			elif self.sdf_check_name_attribute(elem, 'left_disk'):
				train_dict.get('disks')['width'] = float(self.sdf_get_inner_value(self.sdf_get_length(elem)))
				train_dict.get('disks')['radius'] = float(self.sdf_get_inner_value(self.sdf_get_radius(elem)))
				left_disk_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
				train_dict.get('disks')['pose']['left'] = left_disk_pose
				train_dict.get('disks')['coordinates']['left'] = self.pose_to_xyz(left_disk_pose)
			elif self.sdf_check_name_attribute(elem, 'right_disk'):
				right_disk_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
				train_dict.get('disks')['pose']['right'] = right_disk_pose
				train_dict.get('disks')['coordinates']['right'] = self.pose_to_xyz(right_disk_pose)
			elif self.sdf_check_name_attribute(elem, 'left_pad'):
				train_dict.get('pads')['width']['left'] = float(self.sdf_get_inner_value(self.sdf_get_height(elem)))
				left_pad_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
				train_dict.get('pads')['pose']['left'] = left_pad_pose
				train_dict.get('pads')['coordinates']['left'] = self.pose_to_xyz(left_pad_pose)
			elif self.sdf_check_name_attribute(elem, 'right_pad'):
				train_dict.get('pads')['width']['right'] = float(self.sdf_get_inner_value(self.sdf_get_height(elem)))
				right_pad_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
				train_dict.get('pads')['pose']['right'] = right_pad_pose
				train_dict.get('pads')['coordinates']['right'] = self.pose_to_xyz(right_pad_pose)

		return train_dict

	def train_struct_get_pose(self, part):
		return self.train_structure.get(part).get('pose')

	def train_struct_get_width(self, part):
		return self.train_structure.get(part).get('width')

	def train_struct_get_radius(self, part):
		return self.train_structure.get(part).get('radius')
	
	def train_struct_get_coordinates(self, part):
		return self.train_structure.get(part).get('coordinates')

	def train_struct_set_name(self, name):
		self.train_structure['name'] = name

	def train_struct_set_pose(self, part, pose):
		""" Set the pose of the element:
		use this for axis that have only an element.
		Don't use this for wheels, disks and pads. """ 
		self.train_structure.get(part)['pose'] = pose
		self.train_structure.get(part)['coordinates'] = self.pose_to_xyz(pose)

	def train_struct_set_left_pose(self, part, pose):
		""" Set the pose of the left element:
		use this for wheels, disks and pads that have a left and a right element.
		Don't use this for axis """ 
		self.train_structure.get(part)['pose']['left'] = pose
		self.train_structure.get(part)['coordinates']['left'] = self.pose_to_xyz(pose)
	
	def train_struct_set_right_pose(self, part, pose):
		""" Set the pose of the right element:
		use this for wheels, disks and pads that have a left and a right element.
		Don't use this for axis """ 
		self.train_structure.get(part)['pose']['right'] = pose
		self.train_structure.get(part)['coordinates']['right'] = self.pose_to_xyz(pose)

	def train_struct_set_width(self, part, width, dir='none'):
		""" Set the width of the part:
		specify a direction if you want to change the width of only one of the part (for pads, wheels and disks)
		not specifying a direction, the width would be the same for each part."""
		if 'none' in dir:
			self.train_structure.get(part)['width'] = width
		else:
			self.train_structure.get(part)['width'][dir] = width

	def train_struct_set_radius(self, part, radius):
		""" Set the radius for the element "part" in dictionary """
		self.train_structure.get(part)['radius'] = radius

	def xyz_to_pose_string(self, x, y, z):
		""" Given three coordinates (x, y, z), it returns the pose string for sdf """
		return ""+ str(x) + " " + str(y) + " " + str(z) + " " + rotation_string + ""
	
	def pose_to_xyz(self, pose):
		""" Given a pose string (x y z roll pitch yaw), it returns the array of float coordinates """
		str_arr = pose.split(' ')
		return [ float(str_arr[0]), float(str_arr[1]), float(str_arr[2])]

	def pose_to_rpy(self, pose):
		""" GIven a pose string (x y z roll pitch yaw), it returns the array of float rotations """
		str_arr = pose.split(' ')
		return [ float(str_arr[3]), float(str_arr[4]), float(str_arr[5])]

	def train_struct_print(self):
		pprint(self.train_structure)