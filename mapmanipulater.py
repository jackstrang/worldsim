#!/usr/bin/env python

## sends instructions to the mappublisher (which currently is a bad name)

import rospy
#from beginner_tutorials.msg import Tile
'''
def command(x , y):
	pub = rospy.Publisher('commands', Tile, queue_size=10)
	rospy.init_node('commander', anonymous=True)
	rospy.loginfo(x , y)
	pub.publish(x , y)
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		hello_str = instruction % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()
	
'''
from std_msgs.msg import String

def command(text):
	#pub = rospy.Publisher('commands', String, queue_size=10)
	#rospy.init_node('commander', anonymous=True)
	rospy.loginfo(text)
	pub.publish(text)
	'''
	rate = rospy.Rate(.5) # 10hz
	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()
	'''


def highlight_cell():
	cell = raw_input('input xy to highlight: ')
	command(cell)

if __name__ == '__main__':
	
	pub = rospy.Publisher('commands', String, queue_size=10)
	rospy.init_node('commander', anonymous=True)
	try:
		while not rospy.is_shutdown():
			highlight_cell()
	except rospy.ROSInterruptException:
		pass
