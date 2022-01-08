#!/usr/bin/env python3

import rospy
# import math
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

if __name__ == '__main__':
    rospy.init_node('tf_endEffector_node')

    listener = tf.TransformListener() #This will listen to the tf data later

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=100) 

    marker = Marker() 
    marker.id = 0
    marker.header.frame_id = 'base'
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.LINE_STRIP
    marker.ns = 'tf_endEffector_node'
    marker.action = Marker.ADD
    marker.scale.x = .1
    marker.pose.orientation.w = 1.0
    marker.color.a = 1.0
    marker.color.g = 1.0
    
    R = rospy.get_param('~tf_ee_pub_rate')
    rate = rospy.Rate(R)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('base', 'endEffector', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        newpoint = Point()
        newpoint.x = trans[0]
        newpoint.y = trans[1]
        newpoint.z = trans[2]

        if len(marker.points) > 40:
            marker.points.pop(0)        # To make the old trail disappear continuously

        marker.points.append(newpoint)
        
        marker_pub.publish(marker)

        rate.sleep()
