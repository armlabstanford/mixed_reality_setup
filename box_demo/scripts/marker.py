#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3

class MarkerObj:

    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0

        self.orient_x = 0.0
        self.orient_y = 0.0
        self.orient_z = 0.0
        self.orient_w = 1.0

        self.scale_x = 1.0
        self.scale_y = 1.0
        self.scale_z = 1.0

        self.marker_update = rospy.Publisher("marker_topic", Marker, queue_size=10)
        self.pose_subscriber = rospy.Subscriber("pose", Pose, self.handlePoseMsg, queue_size=10)
        self.pose_subscriber = rospy.Subscriber("scale", Vector3, self.handleScaleMsg, queue_size=10)

    def marker_gen(self):
        marker = Marker()

        marker.header.frame_id = "holo_world"
        marker.header.stamp = rospy.Time()

        # IMPORTANT: If you're creating multiple markers, 
        #            each need to have a separate marker ID.
        marker.id = 0

        marker.type = 1 # cube

        marker.pose.position.x = self.pos_x
        marker.pose.position.y = self.pos_y
        marker.pose.position.z = self.pos_z

        marker.pose.orientation.x = self.orient_x
        marker.pose.orientation.y = self.orient_y
        marker.pose.orientation.z = self.orient_z
        marker.pose.orientation.w = self.orient_w

        marker.scale.x = self.scale_x
        marker.scale.y = self.scale_y
        marker.scale.z = self.scale_z

        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        return marker

    def handlePoseMsg(self, data):
        self.pos_x = data.position.x
        self.pos_x = data.position.y
        self.pos_z = data.position.z

        self.orient_x = data.orientation.x
        self.orient_y = data.orientation.y
        self.orient_z = data.orientation.z
        self.orient_w = data.orientation.w

        new_marker = self.marker_gen()
        self.marker_update.publish(new_marker)

    def handleScaleMsg(self, data):
        self.scale_x = data.x
        self.scale_y = data.y
        self.scale_z = data.z

        new_marker = self.marker_gen()
        self.marker_update.publish(new_marker)

if __name__ == '__main__':
    rospy.init_node('marker_node')
    MarkerObj()
    rospy.spin()