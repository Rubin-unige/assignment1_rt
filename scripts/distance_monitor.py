#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from assignment1_rt.msg import turtle_distance
import math

# Global variables to store turtle positions
turtle1_x = 0.0
turtle1_y = 0.0
turtle2_x = 0.0
turtle2_y = 0.0

# Constants for distance and boundary checks
distance_threshold = 2.0
boundary_limit = 1.0
max_limit = 10.0

# Publisher for both turtles' velocity commands
pub_turtle1 = None
pub_turtle2 = None

def turtle1_pose_callback(msg):
    global turtle1_x, turtle1_y
    turtle1_x = msg.x
    turtle1_y = msg.y
def turtle2_pose_callback(msg):
    global turtle2_x, turtle2_y
    turtle2_x = msg.x
    turtle2_y = msg.y

def is_near_boundary(x, y):
    """Check if the turtle is near the boundary."""
    return (x <= boundary_limit or x >= max_limit or y <= boundary_limit or y >= max_limit)

def check_if_overshot_boundary(x, y):
    """Check if the turtle has overshot the boundary."""
    return (x > max_limit or x < boundary_limit or y > max_limit or y < boundary_limit)

def stop_turtle(pub):
    """Stop the turtle by publishing zero velocity."""
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.angular.z = 0.0
    pub.publish(stop_msg)

def main():
    global pub_turtle1, pub_turtle2
    
    rospy.init_node('turtle_distance_monitor')
    
    # Subscribers for turtle positions
    rospy.Subscriber("/turtle1/pose", Pose, turtle1_pose_callback)
    rospy.Subscriber("/turtle2/pose", Pose, turtle2_pose_callback)
    
    # Publishers for controlling turtle velocities
    pub_turtle1 = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=10)
    
    # Publisher for TurtleDistance (custom message)
    pub_distance = rospy.Publisher("/turtle_distance_topic", turtle_distance, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # Calculate the distance between turtles
        distance = math.sqrt((turtle2_x - turtle1_x)**2 + (turtle2_y - turtle1_y)**2)
        
        # Check if turtles are too close
        is_too_close = (distance <= distance_threshold)
        
        # Check if turtles are near boundaries
        turtle1_near_boundary = is_near_boundary(turtle1_x, turtle1_y)
        turtle2_near_boundary = is_near_boundary(turtle2_x, turtle2_y)
        
        # Publish the TurtleDistance custom message
        dist_msg = turtle_distance()
        dist_msg.distance = distance
        dist_msg.is_too_close = is_too_close
        dist_msg.turtle1_near_bound = turtle1_near_boundary
        dist_msg.turtle2_near_bound = turtle2_near_boundary
        pub_distance.publish(dist_msg)
        
        # If turtles are too close or near boundary, stop them
        if is_too_close or turtle1_near_boundary or turtle2_near_boundary:
            if is_too_close:
                stop_turtle(pub_turtle1)  # Stop turtle1
                stop_turtle(pub_turtle2)  # Stop turtle2
                if (distance < distance_threshold):
                    rospy.logwarn("Turtles are really too close!!!")
            
            if turtle1_near_boundary:
                stop_turtle(pub_turtle1)  # Stop turtle1
            
            if turtle2_near_boundary:
                stop_turtle(pub_turtle2)  # Stop turtle2
        
        # After stopping the turtles, check if they have overshot the boundary
        turtle1_overshot = check_if_overshot_boundary(turtle1_x, turtle1_y)
        turtle2_overshot = check_if_overshot_boundary(turtle2_x, turtle2_y)
        
        # Handle overshoot
        if turtle1_overshot:
            rospy.logwarn("Turtle1 is over the boundary after stopping!")
        
        if turtle2_overshot:
            rospy.logwarn("Turtle2 is over the boundary after stopping!")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
