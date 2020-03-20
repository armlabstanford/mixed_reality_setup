#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import Pose

def handlePoseMsg(data):
    #### Start Mapping
    pos_x = data.position.x
    pos_y = data.position.y
    pos_z = data.position.z
    quart_x = data.orientation.x
    quart_y = data.orientation.y
    quart_z = data.orientation.z
    quart_w = data.orientation.w
    
    br = tf.TransformBroadcaster()
    br.sendTransform((pos_x, pos_y, pos_z),
                     (quart_x, quart_y, quart_z, quart_w),
                     rospy.Time.now(),
                     "holo_world",
                     "world")

if __name__ == '__main__':
    rospy.init_node('holoworld_tf_broadcaster')
    rate = rospy.Rate(100)

    pose_subscriber = rospy.Subscriber("holo_world_frame", Pose, handlePoseMsg, queue_size=100)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
