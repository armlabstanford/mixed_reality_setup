#!/usr/bin/env python

# Siemens AG, 2018
# Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# <http://www.apache.org/licenses/LICENSE-2.0>.
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import numpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
	
 
def JoyToTwist():
	# initialize node
	rospy.init_node('JoyToTwist', anonymous=True)

	# setup joy topic subscription
	joy_subscriber = rospy.Subscriber("joy", Joy, handleJoyMsg, queue_size=10)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

	
def handleJoyMsg(data):
	#### Setup Twist Publisher 
   	twist_publisher = rospy.Publisher("cmd_vel", Twist)
	msg = Twist()

	#### Start Mapping from Joy to Twist
	if len(data.axes) >= 2 :	
		msg.linear.y = data.axes[1]*3

	if len(data.axes) >= 1 :
		msg.linear.x = data.axes[0]*3

	#### Publish msg
	rate = rospy.Rate(100) # 10hz
	twist_publisher.publish(msg)
	rate.sleep()

if __name__ == '__main__':
	JoyToTwist()
