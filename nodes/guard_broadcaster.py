#!/usr/bin/env python  
import rospy
import time
import tf_conversions
from nav_msgs.msg import Odometry
import tf2_ros
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion



def handle_guard_pose(msg, guardname):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()


    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = tf_prefix
    t.transform.translation.x = msg.pose.pose.orientation.x
    t.transform.translation.y = msg.pose.pose.orientation.y
    t.transform.translation.z = msg.pose.pose.orientation.z
    w=msg.pose.pose.orientation.w
    orientation_list=[msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,w]
    (a,b,yaw)=euler_from_quaternion(orientation_list)
    t.transform.rotation.z = yaw

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('guard_broadcaster')
    guard1 = rospy.get_param('~tf_prefix', 'robot1_tf')
    guard2 = rospy.get_param('~tf_prefix', 'robot2_tf')
    guard3 = rospy.get_param('~tf_prefix', 'robot3_tf')
    guard4 = rospy.get_param('~tf_prefix', 'robot4_tf')

    rospy.Subscriber('/%s/odom' % guard1,
                     Odometry,
                     handle_guard_pose,
                     guard1)
    rospy.Subscriber('/%s/odom' % guard2,
                     Odometry,
                     handle_guard_pose,
                     guard2)
    rospy.Subscriber('/%s/odom' % guard3,
                     Odometry,
                     handle_guard_pose,
                     guard3)
    rospy.Subscriber('/%s/odom' % guard4,
                     Odometry,
                     handle_guard_pose,
                     guard4)
    rospy.spin()
