#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

def spawn_turtle():
    # Spawn second turtle
    spawn_srv = rospy.ServiceProxy('/spawn', Spawn)
    spawn_srv(5.0, 2.0, 0.0, "turtle2")

def main():
    # Initialize the ROS node
    rospy.init_node('turtle_user_interface', anonymous=True)

    # Publishers for turtle1 and turtle2
    pub_turtle1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    # Spawn turtle2
    spawn_turtle()

    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        # Prompt the user for turtle selection
        turtle_name = input("Enter the turtle you want to control (turtle1 or turtle2): ")

        # Validate the turtle name
        if turtle_name not in ["turtle1", "turtle2"]:
            print("Invalid turtle name. Please enter 'turtle1' or 'turtle2'.")
            continue

        # Get linear and angular velocities with retry logic
        while True:
            try:
                linear_x = float(input("Enter the linear velocity (numerical value): "))
                break
            except ValueError:
                print("Invalid input. Linear velocity must be a number.")

        while True:
            try:
                angular_z = float(input("Enter the angular velocity (numerical value): "))
                break
            except ValueError:
                print("Invalid input. Angular velocity must be a number.")

        # Create and populate the Twist message
        turtle_vel = Twist()
        turtle_vel.linear.x = linear_x
        turtle_vel.angular.z = angular_z

        # Publish the velocity command to the selected turtle
        if turtle_name == "turtle1":
            pub_turtle1.publish(turtle_vel)
        elif turtle_name == "turtle2":
            pub_turtle2.publish(turtle_vel)

        # Move the turtle for 1 second
        rospy.sleep(1.0)

        # Stop the turtle after 1 second
        turtle_vel.linear.x = 0.0
        turtle_vel.angular.z = 0.0
        if turtle_name == "turtle1":
            pub_turtle1.publish(turtle_vel)
        elif turtle_name == "turtle2":
            pub_turtle2.publish(turtle_vel)

        rate.sleep()

if __name__ == "__main__":
    main()
