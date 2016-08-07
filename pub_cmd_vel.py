#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import serial
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def callback(data):
	velocity = Twist()
	current_coords_x = data.x
	current_coords_y = data.y
	current_coords_z = data.z
	
	#dest_coords_y = data.y #input('Enter destination y-coord: ')
	dest_coords_z = data.z
	dist_x = (dest_coords_x - current_coords_x)
	dist_y = (dest_coords_y - current_coords_y)
	dist_z = (dest_coords_z - current_coords_z)
	velocity.linear.x = 0.0002 * (dest_coords_x - current_coords_x)
	velocity.linear.y = 0.0002 * (dest_coords_y - current_coords_y)
	velocity.linear.z = 0.0002 * (dest_coords_z - current_coords_z)
	#velocity.linear.x = 0.1
	#velocity.linear.y = 0.0
	#velocity.linear.z = 0.0
	velocity.angular.x = 0.0
	velocity.angular.y = 0.0
	velocity.angular.z = 0.0
	dist = math.sqrt(math.pow(dist_x, 2) + math.pow(dist_y, 2))
	pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=10)
	pub.publish(velocity)
	xyz_list = [velocity.linear.x, velocity.linear.y, velocity.linear.z]
	rospy.loginfo(xyz_list)
 
def get_location():
	rospy.Subscriber("location", Point, callback)

def location_teller():
	rospy.init_node('pub_cmd_vel', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	global dest_coords_x
	global dest_coords_y
	dest_coords_x = input('Enter destination x-coord: ')
	dest_coords_y = input('Enter destination y-coord: ')
	while not rospy.is_shutdown():
		get_location()
        	rate.sleep()

if __name__ == '__main__':
    try:
        location_teller()
    except rospy.ROSInterruptException:
        pass
