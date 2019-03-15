from xml.dom import minidom
from math import pi
from os.path import expanduser

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
		return self.sdf_train.documentElement.toxml()

	# returns a DOM node for the subelement with tag name "tagname"
	def sdf_get_subelements(self, element, tagname):
		return element.getElementsByTagName(tagname)

	def sdf_get_links(self):
		return self.sdf_get_subelements(self.sdf_train, "link")

	def sdf_get_pose(self, element):
		return self.sdf_get_subelements(element, "pose")
	
	def sdf_get_length(self, element):
		return self.sdf_get_subelements(element, "length")

	def sdf_get_height(self, element):
		return self.sdf_get_subelements(element, "height")

	def sdf_get_radius(self, element):
		return self.sdf_get_subelements(element, "radius")

	# returns the value of the attribute "attr" for the element "element"
	def sdf_get_attribute(self, element, attr):
		return element.getAttribute(attr)

	# returns the value incapsulated by the given tag
	def sdf_get_inner_value(self, element):
		return element[0].firstChild.nodeValue

	def sdf_set_inner_value(self, element, value):
		element.firstChild.nodeValue = value

	# check if attribute "attr" has value "value"
	def	sdf_check_attribute(self, element, attr, value):
		return self.sdf_get_attribute(element, attr) == value

	# check if attribute "name" is "value"
	def sdf_check_name_attribute(self, element, value):
		return self.sdf_check_attribute(element, "name", value)

	# train dictionary management

	# create a dictionary of the type:
	#	{ "name" : "train", "axis" : {"pose": .., "width": .., "radius": ..}, "wheels":{....}...}
	def init_dictionaries(self):
		train_dict = { "name": "train" }

		links = self.sdf_get_links()
		axis_dict = { "name" : "axis"}
		wheels_dict = { "name" : "wheels"}
		disks_dict = { "name" : "disks" }
		pads_dict = { "name" : "pads" }


		for elem in links:
			if self.sdf_check_name_attribute(elem, "axis"):
				axis_dict["width"] = float(self.sdf_get_inner_value(self.sdf_get_length(elem)))
				axis_dict["radius"] = float(self.sdf_get_inner_value(self.sdf_get_radius(elem)))
				axis_dict["pose"] = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
			elif self.sdf_check_name_attribute(elem, "left_wheel"):
				wheels_dict["width"] = float(self.sdf_get_inner_value(self.sdf_get_length(elem)))
				wheels_dict["radius"] = float(self.sdf_get_inner_value(self.sdf_get_radius(elem)))
				lw_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
			elif self.sdf_check_name_attribute(elem, "right_wheel"):
				rw_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
			elif self.sdf_check_name_attribute(elem, "left_disk"):
				disks_dict["width"] = float(self.sdf_get_inner_value(self.sdf_get_length(elem)))
				disks_dict["radius"] = float(self.sdf_get_inner_value(self.sdf_get_radius(elem)))
				ld_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
			elif self.sdf_check_name_attribute(elem, "right_disk"):
				rd_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
			elif self.sdf_check_name_attribute(elem, "left_pad"):
				lp_width = {"left": float(self.sdf_get_inner_value(self.sdf_get_height(elem)))}
				pads_dict["width"] = lp_width
				#pads_dict["radius"] = float(self.sdf_get_inner_value(self.sdf_get_radius(elem)))
				lp_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))
			elif self.sdf_check_name_attribute(elem, "right_pad"):
				pads_dict["width"]["right"] = float(self.sdf_get_inner_value(self.sdf_get_height(elem)))
				rp_pose = str(self.sdf_get_inner_value(self.sdf_get_pose(elem)))

		wheels_dict["pose"] = [lw_pose, rw_pose]
		disks_dict["pose"] = [ld_pose, rd_pose]
		pads_dict["pose"] = [lp_pose, rp_pose]
		
		train_dict["axis"] = axis_dict
		train_dict["wheels"] = wheels_dict
		train_dict["disks"] = disks_dict
		train_dict["pads"] = pads_dict

		return train_dict
	
	def train_struct_get_dict(self, dictName):
		return self.train_structure.get(dictName)

	def train_struct_get_pose(self, dictName):
		return self.train_struct_get_dict(dictName).get("pose")

	def train_struct_get_width(self, dictName):
		return self.train_struct_get_dict(dictName).get("width")

	def train_struct_get_radius(self, dictName):
		return self.train_struct_get_dict(dictName).get("radius")

	def train_struct_get_coordinates(self, dictName):
		pose = self.train_struct_get_pose(dictName)

		if("axis" in dictName):
			pose_arr = pose.split(" ")
			return [pose_arr[0], pose_arr[1], pose_arr[2]]
		else:
			lpose_arr = pose[0].split(" ")
			rpose_arr = pose[1].split(" ")

		return [[lpose_arr[0], lpose_arr[1], lpose_arr[2]],[rpose_arr[0], rpose_arr[1], rpose_arr[2]]]
	
	def train_struct_set_pose(self, dictName, pose):
		self.train_struct_get_dict(dictName)["pose"] = pose

	def train_struct_set_left_pose(self, dictName, pose):
		self.train_struct_get_dict(dictName)["pose"][0] = pose
	
	def train_struct_set_right_pose(self, dictName, pose):
		self.train_struct_get_dict(dictName)["pose"][1] = pose

	def train_struct_set_width(self, dictName, dir, width):
		if "pads" in dictName:
			self.train_struct_get_dict(dictName)["width"][dir] = width
		else:
			self.train_struct_get_dict(dictName)["width"] = width

	def train_struct_set_radius(self, dictName, radius):
		self.train_struct_get_dict(dictName)["radius"] = radius

	# pad management
	def xyz_to_pose_string(self, x, y, z):
		return ""+ str(x) + " " + str(y) + " " + str(z) + " " + rotation_string + ""