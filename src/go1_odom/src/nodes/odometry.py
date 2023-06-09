#!/usr/bin/env python3
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

if __name__ == '__main__':
    rospy.init_node('odom_spoof')
    listener = tf.TransformListener()

    odom_pub = rospy.Publisher('/odometry', Odometry, queue_size=50)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    last_trans = np.array([0., 0., 0.])

    rate = rospy.Rate(60.0)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        try:
            # TODO: are we asking for the right transform
            (trans,rot) = listener.lookupTransform('/odom', '/trunk', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # To np
        trans = np.array(trans)
        rot = np.array(rot)

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "trunk"

        # set the position
        odom.pose.pose = Pose(Point(*trans), Quaternion(*rot))
        odom.pose.covariance = [.0 for _ in range(36)]

        # set the velocity
        # if we want to use this, we should probably average a window
        vth = 0.
        dt = (current_time - last_time).to_sec()
        vel = (trans - last_trans)/dt
        vel = np.array([0., 0.,  0.])  # For now no velocities
        odom.twist.twist = Twist(Vector3(*vel), Vector3(0, 0, vth))
        odom.twist.covariance = [.0 for _ in range(36)]

        # publish the message
        odom_pub.publish(odom)

        last_time = current_time
        last_trans = trans
        last_rot = rot

        rate.sleep()
