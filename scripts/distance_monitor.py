#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from assignment1_rt.msg import turtle_distance
import math

# Global variables to store turtle positions
turtle1_x, turtle1_y, turtle2_x, turtle2_y = 0.0, 0.0, 0.0, 0.0
distance_threshold = 2.0
boundary_limit = 1.0
max_limit = 10.0

# Publisher for both turtles' velocity commands
pub_turtle1 = None
pub_turtle2 = None

# Function to stop the turtles by publishing zero velocity
def stop_turtle(pub):
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.angular.z = 0.0
    pub.publish(stop_msg)

# Callback function for turtle1's pose
def turtle1_pose_callback(msg):
    global turtle1_x, turtle1_y
    turtle1_x = msg.x
    turtle1_y = msg.y

# Callback function for turtle2's pose
def turtle2_pose_callback(msg):
    global turtle2_x, turtle2_y
    turtle2_x = msg.x
    turtle2_y = msg.y

# Function to check if the turtle is near the boundary
def is_near_boundary(x, y):
    return (x < boundary_limit or x > max_limit or y < boundary_limit or y > max_limit)

# Function to check if the turtle has overshot the boundary after stopping
def check_if_overshot_boundary(x, y):
    if x > max_limit or x < boundary_limit or y > max_limit or y < boundary_limit:
        return True
    return False

# Function to reposition the turtle if it overshot the boundary
def reposition_turtle(pub, x, y):
    move_back_msg = Twist()

    # If the turtle has overshot in the x direction
    if x > max_limit:
        rospy.logwarn("Turtle overshot on X axis. Moving back.")
        move_back_msg.linear.x = -0.2  # Move back by applying negative velocity on the X axis
    elif x < boundary_limit:
        rospy.logwarn("Turtle overshot on X axis (negative side). Moving back.")
        move_back_msg.linear.x = 0.2   # Move back by applying positive velocity on the X axis

    # If the turtle has overshot in the y direction
    if y > max_limit:
        rospy.logwarn("Turtle overshot on Y axis. Moving back.")
        move_back_msg.linear.x = -0.2  # Move back in the Y axis
    elif y < boundary_limit:
        rospy.logwarn("Turtle overshot on Y axis (negative side). Moving back.")
        move_back_msg.linear.x = 0.2   # Move back in the Y axis

    # Publish the velocity to move the turtle back
    pub.publish(move_back_msg)

    # Stop the turtle after moving it back within the boundaries
    rospy.sleep(0.5)  # Let the turtle move back for a brief moment
    stop_turtle(pub)  # Stop the turtle after it moves back

def main():
    global pub_turtle1, pub_turtle2

    # Initialize the ROS node
    rospy.init_node("turtle_distance_monitor")

    # Subscribers for turtle positions
    rospy.Subscriber("/turtle1/pose", Pose, turtle1_pose_callback)
    rospy.Subscriber("/turtle2/pose", Pose, turtle2_pose_callback)

    # Publishers for controlling turtle velocities
    pub_turtle1 = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=10)

    # Publisher for TurtleDistance (custom message)
    pub_distance = rospy.Publisher("/turtle_distance_topic", turtle_distance, queue_size=10)

    rate = rospy.Rate(10)  # Set the rate to 10Hz

    while not rospy.is_shutdown():
        # Calculate the distance between turtles
        distance = math.sqrt((turtle2_x - turtle1_x)**2 + (turtle2_y - turtle1_y)**2)

        # Check if turtles are too close
        is_too_close = (distance < distance_threshold)

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

            if turtle1_near_boundary:
                stop_turtle(pub_turtle1)  # Stop turtle1

            if turtle2_near_boundary:
                stop_turtle(pub_turtle2)  # Stop turtle2

        # After stopping the turtles, check if they have overshot the boundary
        turtle1_overshot = check_if_overshot_boundary(turtle1_x, turtle1_y)
        turtle2_overshot = check_if_overshot_boundary(turtle2_x, turtle2_y)

        if turtle1_overshot:
            rospy.logwarn("Turtle1 is over the boundary after stopping!")
            reposition_turtle(pub_turtle1, turtle1_x, turtle1_y)

        if turtle2_overshot:
            rospy.logwarn("Turtle2 is over the boundary after stopping!")
            reposition_turtle(pub_turtle2, turtle2_x, turtle2_y)

        rate.sleep()

if __name__ == "__main__":
    main()
