#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

def pose_publisher():
    pub = rospy.Publisher('/pose_sub', Pose, queue_size=10)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 0
    while not rospy.is_shutdown():
        pose = Pose()

        pose.position.x = 2
        pose.position.y = i
        pose.position.z = 0

        # Make sure the quaternion is valid and normalized
        pose.orientation.x = 0.0
        pose.orientation.x = 0.0
        pose.orientation.x = 0.0
        pose.orientation.w = 1.0

        pub.publish(pose)
        i = i+0.01
        rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass