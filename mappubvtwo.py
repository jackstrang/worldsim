#!/usr/bin/env python

## builds a map by instructions

import rospy
from std_msgs.msg import String

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

import math
import matplotlib.pyplot as plt
import time
import random

tilethickness = .1
roadthickness = .025
auto_road_ratio = .5

class tile:

	def __init__(self, length, width, exitnodelist = [], flyable = True, elevation = 0):
		self.length = length
		self.width = width
		self.exitnodelist = exitnodelist
		self.flyable = flyable
		self.elevation = elevation

	def display(self, coordinate):
		length = self.length
		width = self.width
		base_x = coordinate[0]*length
		base_y = coordinate[1]*width
		xs = [base_x, base_x + length, base_x+length, base_x, base_x]
		ys = [base_y, base_y, base_y+width, base_y+width, base_y]
		plt.plot(xs, ys, 'k')
		if not self.flyable:
			plt.fill(xs, ys, 'r')
		for letter in self.exitnodelist:
			if letter == 'C':
				x = base_x + .5*length
				y = base_y +.5*width
			if letter == 'N':
				x = base_x + .5*length
				y = base_y + width
			if letter == 'S':
				x = base_x + .5*length
				y = base_y
			if letter == 'E':
				x = base_x + length
				y = base_y + .5*width
			if letter == 'W':
				x = base_x
				y = base_y + .5*width
			plt.plot([x, base_x+.5*length],[y, base_y +.5*width], 'b')
			plt.plot([x, base_x+.5*length],[y, base_y +.5*width], 'bo')

class landscape:
	#initiates a landscape
	def __init__(self, num_long, num_wide, tile_dict = {}):
		self.num_long = num_long
		self.num_wide = num_wide
		self.tile_dict = tile_dict
		for x in range(num_long):
			for y in range(num_wide):
				if (x,y) not in tile_dict:
					self.tile_dict[(x, y)] = None

	#fills a tile
	def fill_tile(self, tile, coordinate):
		self.tile_dict[coordinate] = tile

	#displays every tile
	def display(self):
		for co in self.tile_dict.keys():
			t = self.tile_dict[co]
			if t != None:
				t.display(co)
		plt.show(block = False)
			
	#checks if road runs into neighboring tiles, assumes map is valid
	def ground_tile_successors(self, tile_coordinates):
		t = self.tile_dict[tile_coordinates]
		x = tile_coordinates[0]
		y = tile_coordinates[1]
		successors = []
		if 'N' in t.exitnodelist and y<self.num_wide-1:
			successors.append((x, y+1))
		if 'S' in t.exitnodelist and y>0:
			successors.append((x, y-1))
		if 'E' in t.exitnodelist and x<self.num_long-1:
			successors.append((x+1, y))
		if 'W' in t.exitnodelist and x>0:
			successors.append((x-1, y))
		return(successors)

	def get_true_connection_dict(self):
		successor_d = {}
		for co in self.tile_dict:
			suc = self.ground_tile_successors(co)
			successor_d[co] = suc
		checked_d = {}
		for co in successor_d:
			successors = successor_d[co]
			true_successors = []
			for s in successors:
				if co in successor_d[s]:
					true_successors.append(s)
			checked_d[co] = true_successors
		return(checked_d)
	'''
	#checks all 8 (diagonals) adjacent squares to see if flyable
	def sky_tile_successors(self, tile_coordinates):
		if not self.flyable:
			return([])
		t = tile_dict[tile_coordinates]
		x = tile_coordinates[0]
		y = tile_coordinates[1]
		successors = []
		for dx in range(-1, 0, 1):
			for dy in range(-1, 0, 1):
				newx = x+dx
				newy = y+dy
				if newx!=x or newy!=y:
					if (newx, newy) in self.tile_dict.keys():
						if self.tile_dict[(newx, newy)].flyable:
							successors.append((newx, newy))
		return(successors)
	
	#check all tile to see if there is a straight line path there through air
	def sky_tile_successors(self, tile_coordinates):
				t1 = tile_dict[tile_coordinates]
				successors = []
				for t in self.tile_dict.values():
						if t != t1:
								if self.reachable_by_air(self, t, t1):
										successors.append(t)
				return(successors)
	
	#checks single tile to see if there is a straight line path there through air
	def reachable_by_air(self, tile1, tile2):
				#finding inbetween tiles
				x_0 = (tile1.x+.5)*tile1.length
				y_0 = (tile1.y+.5)*tile1.width
		'''
class three_d_tile:
	
	def __init__(self, road_ratio, tile):
		self.road_ratio = road_ratio
		self.tile = tile
	
	def sub_construct(self, co, interactive_marker, hilight = False):

		'''
		rospy.init_node("simple_marker")
	
		# create an interactive marker server on the topic namespace simple_marker
		server = InteractiveMarkerServer("simple_marker")
			
		# create an interactive marker for our server
		int_marker = InteractiveMarker()
		int_marker.header.frame_id = "base_link"
		int_marker.name = "my_marker"
		int_marker.description = "Simple 1-DOF Control"
		'''

		x = (self.tile.length)*co[0]
		y = (self.tile.width)*co[1]

		#base
		base_marker = Marker()
		base_marker.type = Marker.CUBE
		base_marker.scale.x = self.tile.length
		base_marker.scale.y = self.tile.width
		base_marker.scale.z = tilethickness+self.tile.elevation
		base_marker.color.a = 1.0
		if self.tile.flyable:
			base_marker.color.r = 0.0
			base_marker.color.g = 1.0
			base_marker.color.b = 0.0

		else:
			base_marker.color.r = 1.0
			base_marker.color.g = 0.0
			base_marker.color.b = 0.0
		if self.tile.elevation>0:
			base_marker.color.r = .5
			base_marker.color.g = .5
			base_marker.color.b = .5
		if hilight:
			base_marker.color.r = 0.0
			base_marker.color.g = 0.0
			base_marker.color.b = 1.0			
		base_marker.pose.position.x = x
		base_marker.pose.position.y = y
		base_marker.pose.position.z = -.5*tilethickness+(-1*roadthickness)+self.tile.elevation*.5
		
		base_control = InteractiveMarkerControl()
		base_control.always_visible = True
		base_control.markers.append( base_marker )
		interactive_marker.controls.append(base_control)

		#circle
		if len(self.tile.exitnodelist) > 0:
			cylinder_marker = Marker()
			cylinder_marker.type = Marker.CYLINDER
			cylinder_marker.scale.x = self.road_ratio*self.tile.length
			cylinder_marker.scale.y = self.road_ratio*self.tile.width
			cylinder_marker.scale.z = roadthickness
			cylinder_marker.color.r = 0.2
			cylinder_marker.color.g = 0.2
			cylinder_marker.color.b = 0.2
			if hilight:
				cylinder_marker.color.r = 0.0
				cylinder_marker.color.g = 0.0
				cylinder_marker.color.b = 0.4
			cylinder_marker.color.a = 1.0
			cylinder_marker.pose.position.x = x
			cylinder_marker.pose.position.y = y
			cylinder_marker.pose.position.z = -.5*roadthickness+self.tile.elevation

			cylinder_control = InteractiveMarkerControl()
			cylinder_control.always_visible = True
			cylinder_control.markers.append( cylinder_marker )
			interactive_marker.controls.append(cylinder_control)

		#road
		for letter in self.tile.exitnodelist:
			new_x = x
			new_y = y
			new_w = self.tile.width
			new_l = self.tile.length
			l_frac = .08
			w_frac = .08
			if letter != 'C':
				if letter == 'N':
					new_y += self.tile.width*.25
					new_l = self.road_ratio*self.tile.length
					new_w = self.tile.width*.5
					w_frac = .5
				if letter == 'S':
					new_y -= self.tile.width*.25
					new_l = self.road_ratio*self.tile.length
					new_w = self.tile.width*.5
					w_frac = .5
				if letter == 'E':
					new_x += self.tile.length*.25
					new_w = self.road_ratio*self.tile.width
					new_l = self.tile.length*.5
					l_frac = .5
				if letter == 'W':
					new_x -= self.tile.length*.25
					new_w = self.road_ratio*self.tile.width
					new_l = self.tile.length*.5
					l_frac = .5
				road_marker = Marker()
				road_marker.type = Marker.CUBE
				road_marker.scale.x = new_l
				road_marker.scale.y = new_w
				road_marker.scale.z = roadthickness
				road_marker.color.r = 0.2
				road_marker.color.g = 0.2
				road_marker.color.b = 0.2
				if hilight:
					road_marker.color.r = 0.0
					road_marker.color.g = 0.0
					road_marker.color.b = 0.4
				road_marker.color.a = 1.0
				road_marker.pose.position.x = new_x
				road_marker.pose.position.y = new_y
				road_marker.pose.position.z = -.5*roadthickness+self.tile.elevation

				road_control = InteractiveMarkerControl()
				road_control.always_visible = True
				road_control.markers.append( road_marker )
				
				interactive_marker.controls.append(road_control)

				#yellow line
				if len(self.tile.exitnodelist)<3:
					line_marker = Marker()
					line_marker.type = Marker.CUBE
					line_marker.scale.x = new_l*l_frac
					line_marker.scale.y = new_w*w_frac
					line_marker.scale.z = roadthickness*1.15
					line_marker.color.r = 1
					line_marker.color.g = 1
					line_marker.color.b = 0
					line_marker.color.a = 1
					line_marker.pose.position.x = new_x
					line_marker.pose.position.y = new_y
					line_marker.pose.position.z = -.5*roadthickness+self.tile.elevation

					line_control = InteractiveMarkerControl()
					line_control.always_visible = True
					line_control.markers.append( line_marker )

					interactive_marker.controls.append(line_control)

		return(interactive_marker)
		'''
		int_marker.controls.append( total_control )

		server.insert(int_marker, processFeedback)

		# 'commit' changes and send to all clients
		server.applyChanges()

		rospy.spin()
		'''

class three_d_landscape:

	def __init__(self, road_ratio, landscape):
		self.road_ratio = road_ratio
		self.landscape = landscape
		self.tile_dict = {}
		for co in self.landscape.tile_dict.keys():
			t = self.landscape.tile_dict[co]
			three_d_t = three_d_tile(self.road_ratio, t)
			self.tile_dict[co] = three_d_t
		self.num_long = self.landscape.num_long
		self.num_wide = self.landscape.num_wide

	def construct(self, cell_to_hilight = None, path_to_hilight = None, interactive = True, leave = False):
		rospy.init_node("simple_marker")
	
		# create an interactive marker server on the topic namespace simple_marker
		server = InteractiveMarkerServer("simple_marker")
			
		# create an interactive marker for our server
		int_marker = InteractiveMarker()
		int_marker.header.frame_id = "base_link"
		int_marker.name = "my_marker"
		#int_marker.description = "Simple 1-DOF Control"

		for co in self.tile_dict.keys():
			hilight = False
			if co == cell_to_hilight:
				hilight = True
			if path_to_hilight != None:
				if co in path_to_hilight:
					hilight = True
			t = self.tile_dict[co]
			int_marker = t.sub_construct(co, int_marker, hilight)

		server.insert(int_marker, processFeedback)

		# 'commit' changes and send to all clients
		server.applyChanges()
		if interactive:
			self.interact()
		if not leave:
			rospy.spin()

		print('test2')

	def interact(self):
		initial_command = raw_input('Would you like to hilight or search (h or s): ')
		if initial_command == 'h':
			while True:
				cell = input('Input cell as a tuple: ')
				try:
					tup = cell
					break
				except:
					print('Input valid cell')
			#print('hilight_construction')
			self.construct(tup)
		elif initial_command == 's':
			while True:
				cell1 = input('Input start cell as a tuple: ')
				cell2 = input('Input end cell as a tuple: ')
				try:
					tup1 = cell1
					tup2 = cell2
					break
				except:
					print('Inputs were improper, try again')
			path = search_landscape(self.landscape, tup1, tup2)
			self.construct(None, path)
		elif initial_command == 'e':
			return(None)
		self.construct()

class air_column:

	def __init__(self, tile, tile_co, density, height, num_high):
		self.tile = tile
		self.tile_co = tile_co
		self.density = density
		self.height = height
		self.num_high = num_high
		self.node_list = []
		length_between_nodes = self.tile.length/float(self.density)
		width_between_nodes = self.tile.width/float(self.density)
		height_between_nodes = height/float(num_high+1)
		h = int(self.tile.elevation/height_between_nodes)*height_between_nodes
		start_l = (tile_co[0] - .5 + 1/float(2*density))*self.tile.length
		start_w = (tile_co[1] - .5 + 1/float(2*density))*self.tile.width
		while h <= height:
			h += height_between_nodes
			l = 0				
			while l < tile.length:
				w = 0
				while w < tile.width:
					n = (start_l+l, start_w+w, h, self.tile.flyable)
					w += width_between_nodes
					self.node_list.append(n)
				l += length_between_nodes

	def sub_construct(self, int_marker, hilight = False):
		for n in self.node_list:
			n_marker = Marker()
			n_marker.type = Marker.CUBE
			n_marker.scale.x = self.tile.length/float(self.density*10)
			n_marker.scale.y = self.tile.width/float(self.density*10)
			n_marker.scale.z = self.height/float(self.num_high*10)
			if self.tile.flyable:
				n_marker.color.r = 0.0
				n_marker.color.g = 0.0
				n_marker.color.b = 0.5

			else:
				n_marker.color.r = 1.0
				n_marker.color.g = 0.0
				n_marker.color.b = 0.0
			if hilight:
				n_marker.color.r = 0.0
				n_marker.color.g = 0.0
				n_marker.color.b = 1.0			
			n_marker.color.a = .5
			n_marker.pose.position.x = n[0]
			n_marker.pose.position.y = n[1]
			n_marker.pose.position.z = n[2]
			
			n_control = InteractiveMarkerControl()
			n_control.always_visible = True
			n_control.markers.append( n_marker )
			int_marker.controls.append(n_control)

		return(int_marker)



class air_scape:

	def __init__(self, landscape, density, height, num_high):
		self.landscape = landscape
		self.density = density
		self.height = height
		self.num_high = num_high
		self.column_dict = {}
		for t_co in self.landscape.tile_dict.keys():
			t = self.landscape.tile_dict[t_co]
			c = air_column(t, t_co, density, height, num_high)
			self.column_dict[t_co] = c

	def construct(self, cell_to_hilight = None, path_to_hilight = None):
		rospy.init_node("simple_marker")
	
		# create an interactive marker server on the topic namespace simple_marker
		server = InteractiveMarkerServer("simple_marker")
			
		# create an interactive marker for our server
		int_marker = InteractiveMarker()
		int_marker.header.frame_id = "base_link"
		int_marker.name = "my_marker"
		#int_marker.description = "Simple 1-DOF Control"

		for co in self.column_dict.keys():
			hilight = False
			if co == cell_to_hilight:
				hilight = True
			if path_to_hilight != None:
				if co in path_to_hilight:
					hilight = True
			c = self.column_dict[co]
			int_marker = c.sub_construct(int_marker, hilight)

			server.insert(int_marker, processFeedback)

		# 'commit' changes and send to all clients
		server.applyChanges()
		rospy.spin()

class crazyflie:
	def __init__(self, (x, y, z), num, server):
		self.x = x
		self.y = y 
		self.z = z
		self.num = num
		self.marker = InteractiveMarker()
		self.marker.header.frame_id = "crazy_flie_link_"+str(num)
		self.marker.name = "crazy_flie_marker_"+str(num)
		self.marker.description = "Crazy Flie "+str(num)




class system:
	def __init__(self, landscape, road_ratio, density, height, num_high):
		self.landscape = landscape
		self.road_ratio = road_ratio
		self.density = density
		self.height = height
		self.num_high = num_high
		self.landscape3D = three_d_landscape(road_ratio, landscape)
		self.air_scape = air_scape(landscape, density, height, num_high)
		self.true_dict = landscape.get_true_connection_dict()
		self.cell_to_hilight = None

		rospy.init_node("simple_marker")
		rospy.Subscriber('commands', String, self.interpret)

		self.server = InteractiveMarkerServer("simple_marker")
		
		#rospy.Subscriber('commands', String, self.interpret)
		
		
		# create an interactive marker for our server
		self.int_marker = InteractiveMarker()
		self.int_marker.header.frame_id = "base_link"
		self.int_marker.name = "my_marker"

	def construct(self, cell_to_hilight = None, path_to_hilight = None):
		#rospy.init_node("simple_marker")
		'''
		# create an interactive marker server on the topic namespace simple_marker
		self.server = InteractiveMarkerServer("simple_marker")
		
		#rospy.Subscriber('commands', String, self.interpret)
		
		
		# create an interactive marker for our server
		self.int_marker = InteractiveMarker()
		self.int_marker.header.frame_id = "base_link"
		self.int_marker.name = "my_marker"
		#int_marker.description = "Simple 1-DOF Control"
		'''
		self.subconstruct(cell_to_hilight, path_to_hilight)
		self.server.insert(self.int_marker, processFeedback)

			# 'commit' changes and send to all clients
		self.server.applyChanges()
		rospy.spin()


	def subconstruct(self, cell_to_hilight = None, path_to_hilight = None):
		if cell_to_hilight != None:
			self.cell_to_hilight = cell_to_hilight

		for co in self.air_scape.column_dict.keys():
			hilight = False
			if co == self.cell_to_hilight:
				hilight = True
			if path_to_hilight != None:
				if co in path_to_hilight:
					hilight = True
			c = self.air_scape.column_dict[co]
			self.int_marker = c.sub_construct(self.int_marker, hilight)

			#server.insert(int_marker, processFeedback)
		
		for co in self.landscape3D.tile_dict.keys():
			hilight = False
			if co == cell_to_hilight:
				hilight = True
			if path_to_hilight != None:
				if co in path_to_hilight:
					hilight = True
			t = self.landscape3D.tile_dict[co]
			self.int_marker = t.sub_construct(co, self.int_marker, hilight)

	def interpret(self, data):
		print('interpret')
		rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
		hilight_x = int(data.data[0])
		hilight_y = int(data.data[1])
		self.cell_to_hilight = (hilight_x, hilight_y)
		'''
		self.int_marker = None
		self.server.insert(self.int_marker, processFeedback)
		self.server.applyChanges()
		'''
		self.int_marker = InteractiveMarker()
		self.int_marker.header.frame_id = "base_link"
		self.int_marker.name = "my_marker"
		self.subconstruct((hilight_x,hilight_y))
		self.server.insert(self.int_marker, processFeedback)
		self.server.applyChanges()
		rospy.spin()

	def ground_ground_successors(self, tile_co):
		spots = self.true_dict[tile_co]
		suc = []
		for s in spots:
			c = co_distance(s, tile_co)
			suc.append((s, c))
		return(suc)

	def ground_air_successors(self, tile_co):
		t = 1

	def air_ground_successors(self, node_co):
		t = 1

	def air_air_successors(self, node_co):
		t = 1


def run_full_search(l):
	pairs = []
	for co1 in l.tile_dict.keys():
		for co2 in l.tile_dict.keys():
			if co1 != co2:
				pairs.append((co1, co2))
	random.shuffle(pairs)
	for p in pairs:
		co1 = p[0]
		co2 = p[1]
		path = search_landscape(l, co1, co2)
		print(str(co1)+" to "+str(co2))
		l_3D = three_d_landscape(.75, l)
		l_3D.construct(None, path, False, True)
		time.sleep(5)


	'''
	def construct_and_run(self):
		going = True
		while going:
			self.construct()
			print('hello')
			cell_to_hilight = raw_input('Hello there!')
	'''
'''
#unrelated code, demonstration of a basic publisher
def talker():
	x = raw_input('test')
	pub = rospy.Publisher('chatter', String, queue_size=10)
	rospy.init_node('mappub', anonymous=True)
	rate = rospy.Rate(1) # 10hz
	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()
'''
def callback(data):
	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
	x = int(data.data[0])
	y = int(data.data[1])




#user interface to construct the landscape
def builder():
	tile_length = input('input tile length: ')
	tile_width = input('input tile width: ')
	num_long = input('input num long: ')
	num_wide = input('input num wide: ')
	l = landscape(num_long, num_wide)
	for x in range(num_long):
		for y in range(num_wide):
			proper = False
			while not proper:
				print('current tile is '+str((x, y)))
				directions = raw_input('input directions: ')
				exitnodelist = []
				if 'c' in directions or 'C' in directions:
					exitnodelist.append('C')
				if 'n' in directions or 'N' in directions:
					exitnodelist.append('N')
				if 's' in directions or 'S' in directions:
					exitnodelist.append('S')
				if 'e' in directions or 'E' in directions:
					exitnodelist.append('E')
				if 'w' in directions or 'W' in directions:
					exitnodelist.append('W')
				flyable = raw_input('input flyable: ')
				if flyable == 'True' or flyable == 'False':
					flyable = bool(flyable)
					proper = True
				else:
					print('Something went wrong, try again on the same tile')
			print(" ")
			t = tile(tile_length, tile_width, exitnodelist, flyable)
			l.fill_tile(t, (x, y))
			l.display()
	l_3D = three_d_landscape(auto_road_ratio, l)
	if __name__=="__main__":
		l_3D.construct()
	connection_dict = l.get_true_connection_dict()
	return(connection_dict)

#calculates distance between center of tile instances, will be useful as a cost function
def distance(tile1, tile2):
		dsquared = (tile1.length*(tile1.x-tile2.x))**2+(tile1.width*(tile1.y-tile2.y))**2
		return(dsquared**.5)

def co_distance(co1, co2):
	if len(co1) == 2:
		co1 += (0)
	if len(co2) == 2:
		co2 += (0)
	d = ((co1[0]-co2[0])**2+(co1[1]-co2[1])**2+(co1[2]-co2[2])**2)**.5
	return(d)


def processFeedback(feedback):
	p = feedback.pose.position
	print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)



'''
def processFeedback(feedback):
	p = feedback.pose.position
	print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

'''

#rviz code not related, just for looking at
'''
if __name__=="__main__":
	rospy.init_node("simple_marker")
	
	# create an interactive marker server on the topic namespace simple_marker
	server = InteractiveMarkerServer("simple_marker")
	
	# create an interactive marker for our server
	int_marker = InteractiveMarker()
	int_marker.header.frame_id = "base_link"
	int_marker.name = "my_marker"
	int_marker.description = "Simple 1-DOF Control"

	# create a grey box marker
	box_marker = Marker()
	box_marker.type = Marker.CUBE
	box_marker.scale.x = 0.45
	box_marker.scale.y = 0.45
	box_marker.scale.z = 0.15
	box_marker.color.r = 0.0
	box_marker.color.g = 0.5
	box_marker.color.b = 0.5
	box_marker.color.a = 1.0
	box_marker.pose.position.x = 0.5
	box_marker.pose.position.y = 0.5
	box_marker.pose.position.z = 1.0

	cylinder_marker = Marker()
	cylinder_marker.type = Marker.CYLINDER
	cylinder_marker.scale.x = .45
	cylinder_marker.scale.y = .45
	cylinder_marker.scale.z = 1
	cylinder_marker.color.r = 1
	cylinder_marker.color.g = .5
	cylinder_marker.color.b = 0
	cylinder_marker.color.a = 1.0

	# create a non-interactive control which contains the box
	box_control = InteractiveMarkerControl()
	box_control.always_visible = True
	box_control.markers.append( box_marker )

	cylinder_control = InteractiveMarkerControl()
	cylinder_control.always_visible = True
	cylinder_control.markers.append( cylinder_marker )
	# add the control to the interactive marker
	int_marker.controls.append( box_control )
	int_marker.controls.append( cylinder_control )


	# create a control which will move the box
	# this control does not contain any markers,
	# which will cause RViz to insert two arrows
	rotate_control = InteractiveMarkerControl()
	rotate_control.name = "move_x"
	rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

	# add the control to the interactive marker
	#int_marker.controls.append(rotate_control);

	# add the interactive marker to our collection &
	# tell the server to call processFeedback() when feedback arrives for it
	server.insert(int_marker, processFeedback)

	# 'commit' changes and send to all clients
	server.applyChanges()

	rospy.spin()


'''


##search

##this will search through the dictionary returned from a landscape in the 
##get_true_connection_dict function

class SearchNode:
	def __init__(self, state, parent, cost=0):
		self.state = state
		self.parent = parent
		self.cost = cost

	def path(self):
		if self.parent == None:
			return [self.state]
		else:
			return self.parent.path() + [self.state]
	
class PriorityQueue:
	def __init__(self):
		self.data = []
	def push(self, item, cost):
		self.data.append((cost, item))
	def pop(self):
		self.data.sort()
		return self.data.pop(0)[1]
	def is_empty(self):
		return len(self.data) == 0
 
def a_star(successors, start_state, goal_test, heuristic=lambda x: 0):
	if goal_test(start_state):
		return [start_state]
	start_node = SearchNode(start_state, None, 0)
	agenda = PriorityQueue()
	agenda.push(start_node, heuristic(start_state))
	expanded = set()
	while not agenda.is_empty():
		parent = agenda.pop()
		if parent.state not in expanded:
			expanded.add(parent.state)
			if goal_test(parent.state):
				return parent.path()
			for child_state, cost in successors(parent.state):
				child = SearchNode(child_state, parent, parent.cost+cost)
				if child_state in expanded:
					continue
				agenda.push(child, child.cost+heuristic(child_state))
	return None


def search_landscape(landscape, start_coordinate, end_coordinate):
	true_dict = landscape.get_true_connection_dict()
	def successors(coordinate):
		spots = true_dict[coordinate]
		suc = []
		for s in spots:
			suc.append((s, 1))
		return(suc)
	def goal_test(coordinate):
		return coordinate == end_coordinate
	def dist(coordinate):
		distance = ((coordinate[0] - end_coordinate[0])**2 + (coordinate[1] - end_coordinate[1])**2)**.5
		return(distance)
	return(a_star(successors, start_coordinate, goal_test, dist))
    







#whats actually being run



'''
#runs the building code
if __name__ == '__main__':
	try:
		builder()
	except rospy.ROSInterruptException:
		pass
'''

#encoded duckytown
duckytown_num_long = 6
duckytown_num_wide = 10
duckytown_tile_size = 1
non_fly_list = [(0,2),(0,3),(0,4),(0,5)]
duckytown_pre_dict = {(0,0):['N','E'], (0,1):['S','E'], (0,2): [], (0,3):[], 
(0,4):[], (0,5):[], (0,6):['N','E'], (0,7):['S','N'], (0,8):['S','N'], (0,9):['S','E'], (1,0):['E','W'], (1,1):['W','N'],(1,2):['S','N'], (1,3):['S','N'], (1,4):['S','E'], (1,5):[], (1,6):['E','W','N'], (1,7):['S','N'], (1,8):['S'], (1,9):['E','W'], (2,0):['W','E','N'], (2,1):['S','N'], (2,2):['S','E'], (2,3):[], (2,4):['E','W'], (2,5):[], (2,6):['W','N'], (2,7):['S','N'], (2,8):['S','N'], (2,9):['S','E','W'], (3,0):['E','W'], (3,1):[], (3,2):['W','N'], (3,3):['N','S'], (3,4):['N','S','E','W'], (3,5):['N','S'], (3,6):['N','S'], (3,7):['S','E'],(3,8):[], (3,9):['E','W'], (4,0):['E','W'], (4,1):[], (4,2):['C'], (4,3):[], (4,4):['E','W'], (4,5):[], (4,6):[], (4,7):['E','W'], (4,8):[], (4,9):['E','W'], (5,0):['W','N'], (5,1):['S','N'], (5,2):['S','N'], (5,3):['S','N'], (5,4):['S','N','W'], (5,5):['S','N'], (5,6):['S','N'], (5,7):['S','N','W'], (5,8):['S','N'], (5,9):['S','W']}
duckytown_dict = {}
for co in duckytown_pre_dict.keys():
	exitnodelist = duckytown_pre_dict[co]
	flyable = True
	elevation = 0
	if 'C' in exitnodelist:
		elevation = 3
	if co in non_fly_list:
		flyable = False
	t = tile(duckytown_tile_size, duckytown_tile_size, exitnodelist, flyable, elevation)
	duckytown_dict[co] = t
duckytown = landscape(duckytown_num_long, duckytown_num_wide, duckytown_dict)
#run_full_search(duckytown)
#duckytown.display()
path = search_landscape(duckytown, (1,1), (2,1))
print(path)
dic = (duckytown.get_true_connection_dict())
#print(len(duckytown.get_true_connection_dict()))
duckytown3D = three_d_landscape(.75, duckytown)
#duckytown3D.construct()
duckytownAir = air_scape(duckytown, 3, 5, 5)
duckytownSystem = system(duckytown, .75, 1, 1, 1)
if __name__=="__main__":
	duckytownSystem.construct()
	#duckytown3D.construct(None, None, False)
print('test')
