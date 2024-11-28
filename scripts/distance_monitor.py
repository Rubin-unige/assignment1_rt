#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from assignment1_rt.msg import turtle_distance
import math

# Global variables to store turtle positions
turtle1_x, turtle1_y, turtle2_x, turtle2_y, turtle2_theta, turtle1_theta = 0, 0, 0, 0, 0, 0
prev_turtle1_x, prev_turtle1_y, prev_turtle2_x, prev_turtle2_y = 0, 0, 0, 0
distance_threshold = 1.0
boundary_limit = 1.0
max_limit = 10.0

# Publisher for turtle velocity commands
pub_turtle1, pub_turtle2 = None, None

def turtle1_pose_callback(msg):
    global turtle1_x, turtle1_y, turtle1_theta
    turtle1_x = msg.x
    turtle1_y = msg.y
    turtle1_theta = msg.theta
    rospy.loginfo("Turtle1 updated position: (%.2f, %.2f, %.2f)", turtle1_x, turtle1_y, turtle1_theta)

def turtle2_pose_callback(msg):
    global turtle2_x, turtle2_y, turtle2_theta
    turtle2_x = msg.x
    turtle2_y = msg.y
    turtle2_theta = msg.theta
    rospy.loginfo("Turtle2 updated position: (%.2f, %.2f, %.2f)", turtle2_x, turtle2_y, turtle2_theta)

def is_near_boundary(x, y):
    return x < boundary_limit or x > max_limit or y < boundary_limit or y > max_limit

def check_if_overshot_boundary(x, y):
    return x > max_limit or x < boundary_limit or y > max_limit or y < boundary_limit

def stop_turtle(pub):
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.angular.z = 0.0
    pub.publish(stop_msg)

def reposition_turtle(pub, x, y, prev_y, theta):
    move_back_msg = Twist()
    is_moving_up = y > prev_y
    is_moving_down = y < prev_y
    same_value = y == prev_y

    # Handling Y-axis overshooting
    if y > max_limit:
        move_back_msg.linear.x = -0.2 if theta >= 0 else 0.2
    elif y < boundary_limit:
        move_back_msg.linear.x = 0.2 if theta > 0 else -0.2

    # Handling X-axis overshooting
    if x > max_limit:
        if theta > 0:
            move_back_msg.linear.x = -0.2 if is_moving_up else 0.2
        elif theta == 0:
            move_back_msg.linear.x = -0.2
        elif theta == -3.14:
            move_back_msg.linear.x = 0.2
    elif x < boundary_limit:
        if theta >= 0:
            move_back_msg.linear.x = -0.2 if is_moving_up or same_value else 0.2
        elif theta == 0:
            move_back_msg.linear.x = 0.2
        elif theta == -3.14:
            move_back_msg.linear.x = -0.2

    pub.publish(move_back_msg)
    rospy.sleep(0.5)
    stop_turtle(pub)

def main():
    global pub_turtle1, pub_turtle2, prev_turtle1_x, prev_turtle1_y, prev_turtle2_x, prev_turtle2_y

    rospy.init_node('turtle_distance_monitor', anonymous=True)

    rospy.Subscriber("/turtle1/pose", Pose, turtle1_pose_callback)
    rospy.Subscriber("/turtle2/pose", Pose, turtle2_pose_callback)

    pub_turtle1 = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=10)
    pub_distance = rospy.Publisher("/turtle_distance_topic", turtle_distance, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        distance = math.sqrt((turtle2_x - turtle1_x)**2 + (turtle2_y - turtle1_y)**2)
        is_too_close = distance <= distance_threshold
        turtle1_near_boundary = is_near_boundary(turtle1_x, turtle1_y)
        turtle2_near_boundary = is_near_boundary(turtle2_x, turtle2_y)

        dist_msg = turtle_distance()
        dist_msg.distance = distance
        dist_msg.is_too_close = is_too_close
        dist_msg.turtle1_near_bound = turtle1_near_boundary
        dist_msg.turtle2_near_bound = turtle2_near_boundary
        pub_distance.publish(dist_msg)

        if is_too_close or turtle1_near_boundary or turtle2_near_boundary:
            if is_too_close:
                stop_turtle(pub_turtle1)
                stop_turtle(pub_turtle2)
                rospy.logwarn("Turtles are too close!")

            if turtle1_near_boundary:
                stop_turtle(pub_turtle1)
            if turtle2_near_boundary:
                stop_turtle(pub_turtle2)

        turtle1_overshot = check_if_overshot_boundary(turtle1_x, turtle1_y)
        turtle2_overshot = check_if_overshot_boundary(turtle2_x, turtle2_y)

        if turtle1_overshot:
            rospy.logwarn("Turtle1 overshot the boundary!")
            reposition_turtle(pub_turtle1, turtle1_x, turtle1_y, prev_turtle1_y, turtle1_theta)
        if turtle2_overshot:
            rospy.logwarn("Turtle2 overshot the boundary!")
            reposition_turtle(pub_turtle2, turtle2_x, turtle2_y, prev_turtle2_y, turtle2_theta)

        prev_turtle1_x, prev_turtle1_y = turtle1_x, turtle1_y
        prev_turtle2_x, prev_turtle2_y = turtle2_x, turtle2_y

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
