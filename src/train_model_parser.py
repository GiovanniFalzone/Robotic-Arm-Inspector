from xml.dom import minidom
from math import pi

MODELS_DIR = "../models/"
rotation_string = "0.0 " + str(pi/2) + " " + str(pi/2) +" "

class train_model():
	def __init__(self):
		self.sdf_train = minidom.parse(MODELS_DIR + "train_model/model.sdf")
		self.x = 0.0
		self.y = 0.0
		self.z = 2.0

	def get_sdf(self):
		return self.sdf_train.documentElement.toxml()

	def get_links(self):
		return self.sdf_train.getElementsByTagName("link")

	def get_attribute(self, element, attr):
		return element.getAttribute(attr)

	def get_value(self, element, tag):
		return element.getElementsByTagName(tag)

	def get_pose(self, element):
		return self.get_value(element, "pose")
	
	def get_length(self, element):
		return self.get_value(element, "length")
	
	def get_inner_value(self, element):
		for sub_el in element:
			return sub_el.firstChild.nodeValue

	def	check_attribute(self, element, attr, value):
		return self.get_attribute(element, attr) == value

	def check_name_attribute(self, element, value):
		return self.check_attribute(element, "name", value)

	def set_value(self, element, value):
		element.firstChild.nodeValue = value
	
	def find_pad_position(self, dir, pw):
		y = 0
		links = self.get_links()
		axis_width = 0
		disk_width = 0

		for elem in links:
			if self.check_name_attribute(elem, "axis"):
				axis_width = float(self.get_inner_value(self.get_length(elem)))

			elif self.check_name_attribute(elem, dir+"_disk"):
				disk_width = float(self.get_inner_value(self.get_length(elem)))
				
		if "left" in dir:
			y = axis_width / 3 + disk_width / 2 + pw / 2
		elif "right" in dir:
			y = 2*axis_width/3 - disk_width / 2 - pw / 2
		else:
			print "wrong direction given"

		return [self.x, y, self.z]

	def get_pad_position(self, dir, pw):
		return self.find_pad_position(dir, pw)

	def set_pads_width(self, left_pad_width, right_pad_width):
		links = self.get_links()

		#iterate on links
		for elem in links:
			# check if name attribute is "left_pad" or "right_pad"
			if self.check_name_attribute(elem, "left_pad"):
				# get y coordinates for left pad
				p_coord = self.find_pad_position("left", left_pad_width)
				pad_pose_value =  ""+ str(p_coord[0]) + " " + str(p_coord[1]) + " " + str(p_coord[2]) + " " + rotation_string + ""
				pad_pose = self.get_pose(elem)
				
				# set left pad position
				self.set_value(pad_pose[0], pad_pose_value)
			
				pad_width = self.get_length(elem)
				#set left pad width w.r.t. its position
				for pad_l in pad_width:
					self.set_value(pad_l, left_pad_width)
			
			elif self.check_name_attribute(elem, "right_pad"):
				p_coord = self.find_pad_position("right", right_pad_width)
				pad_pose_value =  ""+ str(p_coord[0]) + " " + str(p_coord[1]) + " " + str(p_coord[2]) + " " + rotation_string + ""
				pad_pose = self.get_pose(elem)

				self.set_value(pad_pose[0], pad_pose_value)

				pad_width = self.get_length(elem)
				for pad_l in pad_width:
					self.set_value(pad_l, right_pad_width)
				
			

