#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates

def getCurrentPose():
	# setup topic subscription
	currentpose_subscriber = rospy.Subscriber("/gazebo/get_model_state", ModelState, saveCurrentPose, queue_size=10)

def updatePose():
	# initialize node
	rospy.init_node('UpdatePose', anonymous=True)
	
	# setup topic subscription
	point_subscriber = rospy.Subscriber("point", Point, handlePointMsg, queue_size=10)
	
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

	
def handlePointMsg(data):
	#### Setup ModelState Publisher 
   	modelstate_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)

	#### Start Mapping
	currentPose.model_name = "box"
	currentPose.pose.position.x = data.x
	currentPose.pose.position.y = data.y
	currentPose.pose.position.z = data.z

	#### Publish msg
	rate = rospy.Rate(100) # 10hz
	modelstate_publisher.publish(currentPose)
	rate.sleep()

def saveCurrentPose(data):
	currentPose = data

if __name__ == '__main__':
	currentPose = ModelState()	
	getCurrentPose()	
	updatePose()
