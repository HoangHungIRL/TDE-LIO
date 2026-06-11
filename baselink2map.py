#!/usr/bin/env python3  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
# import turtlesim.srv

if __name__ == '__main__':
    # current_time = rospy.Time.now()
    rospy.init_node('baselink2map_tf_listener')
    odom_pub = rospy.Publisher("base2map", Odometry, queue_size=50)
    # path_pub = rospy.Publisher("path_length", Float64, queue_size=50)
    listener = tf.TransformListener()
    # (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
    first_time = True 
    # prev_x = 0
    # prev_y = 0
    total_path_length = 0.0
    path_lenght = 0.0 
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('map', 'chassis_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = rospy.Time(0)
        odom.header.frame_id = "map"
        # print("trans: "+ str(trans))
        # print("rot:" + str(rot))
        # set the position
        odom.pose.pose.position.x = trans[0]
        odom.pose.pose.position.y = trans[1]
        odom.pose.pose.position.z = trans[2]

        odom.pose.pose.orientation.x = rot[0]
        odom.pose.pose.orientation.y = rot[1]
        odom.pose.pose.orientation.z = rot[2]
        odom.pose.pose.orientation.w = rot[3]

        # ## path 
        # path = Float64()
        # x = round(odom.pose.pose.position.x,3)
        # y = round(odom.pose.pose.position.y,3)
        # if first_time:
        #     first_time = False 
        #     prev_x = x
        #     prev_y = y 
        
        # else:

        #     distance_2_points = math.hypot(prev_x - x, prev_y - y) 
        #     if distance_2_points >= 0.2:
        #         path_lenght +=distance_2_points
        #         # if  prev_x - x > 0.1 or (prev_y - y) > 0.1:
        #         prev_x = x
        #         prev_y = y
        #         rospy.sleep(0.1)
        #         total_path_length  += path_lenght
        #     else:  
        #         total_path_length  += 0 
        #     path = total_path_length
        #     print('path length:' + str(path))
        #     path_pub.publish(path)

        # set the velocity
        odom.child_frame_id = "chassis_link"
        # odom.twist.twist = rot

        # publish the message
        odom_pub.publish(odom)
        
        rate.sleep()
