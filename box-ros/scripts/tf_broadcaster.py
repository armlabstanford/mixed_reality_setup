#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import Point

def main():
    while not rospy.is_shutdown():
        # setup topic subscription
        point_subscriber = rospy.Subscriber("point", Point, handlePointMsg, queue_size=100)

        rate.sleep()

def handlePointMsg(data):
    #### Start Mapping
    pose_x = data.x
    pose_y = data.y
    pose_z = data.z

    br = tf.TransformBroadcaster()
    br.sendTransform((pose_x, pose_y, pose_z),
                     (0, 0, 0, 1),
                     rospy.Time.now(),
                     "box",
                     "world")

if __name__ == '__main__':
    rospy.init_node('box_tf_broadcaster')
    rate = rospy.Rate(100)

    pose_x = 1
    pose_y = 0
    pose_z = 0

    point_subscriber = rospy.Subscriber("point", Point, handlePointMsg, queue_size=100)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()