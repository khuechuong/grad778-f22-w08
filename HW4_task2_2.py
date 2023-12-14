#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float64

# Initialize the angular tolerance


def dist_cal(data):
    vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg  = Twist()
    rospy.loginfo(data.data)

    # Since the Wall-E moving just in x-axis
    # vel_msg.linear.x = speed
    vel_msg.linear.x = 0.2
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0 
    vel_msg.angular.z = 0

    # Setting the starting time
    t0 = rospy.Time.now().to_sec()

    current_dist = 0

    while current_dist < int(desired_dist):
        # Publish the velocity
        vel_pub.publish(vel_msg)

        # Calculate the current time
        
        t1 = rospy.Time.now().to_sec()


        # Calculate the current distance
        current_dist = vel_msg.linear.x*(t1-t0)

    # Stop the robot
    vel_msg.linear.x = 0
    vel_pub.publish(vel_msg)


while not rospy.is_shutdown():

    # Initialize a sub nod
    rospy.init_node("subcriber", anonymous=True)

    # Command the desired distance as an input in terminal
    desired_dist = input("Let's go Wall-E:")

    # Subscribe the distance error
    distSub = rospy.Subscriber("/distance_separation", Float64, dist_cal)

    rospy.spin()











    


