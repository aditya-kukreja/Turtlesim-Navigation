#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


from math import atan2, pow, sqrt, asin, cos
import math
import time

p0=Pose()
p1=Pose()
p1.x=7.5
p1.y=5.544445
p2=Pose()
p2.x=6.2
p2.y=3.5
p3=Pose()
p3.x=4.3
p3.y=7.5

l1=LaserScan()












def clbk(msg):
    global p0
    p0=msg

def laser_reading():
    i=p0.theta-(math.pi*(3/4))
    global d1,theta1,d2,theta2,d3,theta3
    for j in range(0, 540):          
         # l1.ranges.append(float('inf'))
          d1=sqrt(pow((p1.x-p0.x),2)+pow((p1.y-p0.y),2))
          d2=sqrt(pow((p2.x-p0.x),2)+pow((p2.y-p0.y),2))
          d3=sqrt(pow((p3.x-p0.x),2)+pow((p3.y-p0.y),2))
          theta1=atan2(p1.y-p0.y,p1.x-p0.x)
          theta2=atan2(p2.y-p0.y,p2.x-p0.x)
          theta3=atan2(p3.y-p0.y,p3.x-p0.x)
          l1.ranges[j]=float('inf') 
          if i>=theta1-asin(0.5/d1) and i<=theta1+asin(0.5/d1):
             l1.ranges[j]=d1*cos(theta1-i)-sqrt((d1*d1*pow(cos(theta1-i),2))+(0.5*0.5)-(d1*d1))
          if i>=theta2-asin(0.5/d2) and i<=theta2+asin(0.5/d2):
             l1.ranges[j]=min(d2*cos(theta2-i)-sqrt((d2*d2*pow(cos(theta2-i),2))+(0.5*0.5)-(d2*d2)),l1.ranges[j])
          if i>=theta3-asin(0.5/d3) and i<=theta3+asin(0.5/d3):
             l1.ranges[j]=min(d3*cos(theta3-i)-sqrt((d3*d3*pow(cos(theta3-i),2))+(0.5*0.5)-(d3*d3)),l1.ranges[j])
          
          i=i+(0.5*(math.pi/180)) 
    

def main():
    global p0, l1, p1, p2, p3, d1, theta1,d2,d3,theta2,theta3
    rospy.init_node('laserscanner') 
    sub_pose=rospy.Subscriber('/turtle1/pose',Pose,clbk)
    pub_laser=rospy.Publisher('/scan',LaserScan,queue_size=10)
    for j in range(0,540):
           l1.ranges.append(float('inf'))

    r = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        l1.header.stamp = current_time
        l1.header.frame_id = 'base_link'
        l1.angle_min = -2.355
        l1.angle_max = 2.355
        l1.angle_increment = 4.71 / 540
        l1.time_increment = (1.0 / 40) / (540)
        l1.range_min = 0.0
        l1.range_max = float('inf')

        laser_reading()
        pub_laser.publish(l1)
        for j in range(0,540):
           if l1.ranges[j]!=float('inf'):
              print(j)
            #print(d1)
            #print(theta1)
              print(l1.ranges[j])
              
        r.sleep()    
        
    
if __name__ == '__main__':
   main()
     
          
