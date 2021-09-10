#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from turtlesim.msg import Pose

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

global p0, v0
p0= Pose()
v0= Twist()
def clbk_pose(pos_msg):
    global p0
    p0=pos_msg
    

def clbk_vel(vel_msg):
    global v0
    v0=vel_msg
    
#x = p0.x
#y = p0.y
#th = p0.theta

#vx = v0.linear.x
#vy = v0.linear.y
#vth = v0.angular.z






current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(20.0)


def main():
    sub_Pose=rospy.Subscriber('/turtle1/pose', Pose, clbk_pose)
    sub_vel=rospy.Subscriber('/turtle1/cmd_vel', Twist, clbk_vel)
    while not rospy.is_shutdown():
    
        current_time = rospy.Time.now()
        print(v0.linear.x)
        print(p0.y)
    # compute odometry in a typical way given the velocities of the robot
    #dt = (current_time - last_time).to_sec()
    #delta_x = (vx * cos(th) - vy * sin(th)) * dt
    #delta_y = (vx * sin(th) + vy * cos(th)) * dt
    #delta_th = vth * dt

    #x += delta_x
    #y += delta_y
    #th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, p0.theta)

    # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (p0.x, p0.y, 0.0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

    # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

    # set the position
        odom.pose.pose.position=(Point(p0.x, p0.y, 0.0))
        odom.pose.pose.orientation=Quaternion(*odom_quat)

    # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(v0.linear.x, v0.linear.y, 0), Vector3(0, 0, v0.angular.z))

    # publish the message
        odom_pub.publish(odom)

        last_time = current_time
        r.sleep()
  
if __name__ == '__main__':
   main()        
        
