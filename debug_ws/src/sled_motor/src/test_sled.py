#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg 			import *
from arm_sanding_utilities.msg import SledPosition


rospy.init_node("test_node", anonymous=True,disable_signals=True)

sled_pos_pub = rospy.Publisher('/sled/cmd_position', SledPosition, queue_size=10)
msg = SledPosition()
msg.position = 0.1
rate = rospy.Rate(10)
ctrl_c = False

while not ctrl_c:
	connections = sled_pos_pub.get_num_connections()
	if connections > 0:
		sled_pos_pub.publish(msg)
		ctrl_c = True
		print("Message published")
	else:
		rate.sleep()

