#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


def check_if_turtle2_exists():
    """Checks if turtle2 exists by subscribing to /turtle2/pose."""
    turtle2_exists = False

    def pose_callback(msg):
        nonlocal turtle2_exists
        turtle2_exists = True  # Message received; turtle2 exists

    # Subscribe to /turtle2/pose topic
    rospy.Subscriber('/turtle2/pose', Pose, pose_callback)

    # Short delay, wait for message
    timeout_time = rospy.Time.now() + rospy.Duration(1.0)  # 1-second timeout
    while rospy.Time.now() < timeout_time and not rospy.is_shutdown():
        rospy.sleep(0.1)  # Sleep in small increments

    return turtle2_exists


def spawn_turtle():
    """Spawns turtle2 if it doesn't exist."""
    rospy.loginfo("Spawning Turtle2.")
    try:
        rospy.wait_for_service('/spawn', timeout=5)
        spawn_srv = rospy.ServiceProxy('/spawn', Spawn)
        spawn_srv(5.0, 2.0, 0.0, "turtle2")
    except rospy.ROSException as e:
        rospy.logerr(f"Spawn service not available: {e}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn turtle2: {e}")


def main():
    # Initialize ROS node
    rospy.init_node('turtle_user_interface', anonymous=True)

    # Publishers for turtle1 and turtle2
    pub_turtle1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    # Check if turtle2 exists and spawn if necessary
    if check_if_turtle2_exists():
        rospy.loginfo("Turtle2 already exists.")
    else:
        spawn_turtle()

    rate = rospy.Rate(10)  # Loop rate

    while not rospy.is_shutdown():
        # Prompt user for turtle selection
        turtle_name = input("Enter the turtle you want to control (turtle1 or turtle2): ")

        # Validate turtle name
        if turtle_name not in ["turtle1", "turtle2"]:
            print("Invalid turtle name. Please enter 'turtle1' or 'turtle2'.")
            continue

        # Get linear velocity with range validation
        while True:
            try:
                linear_x = float(input("Enter the linear velocity (between -5 and 5): "))
                if -5 <= linear_x <= 5:
                    break
                else:
                    print("Invalid input. Linear velocity must be between -5 and 5.")
            except ValueError:
                print("Invalid input. Linear velocity must be a number.")

        # Get angular velocity with range validation
        while True:
            try:
                angular_z = float(input("Enter the angular velocity (between -5 and 5): "))
                if -5 <= angular_z <= 5:
                    break
                else:
                    print("Invalid input. Angular velocity must be between -5 and 5.")
            except ValueError:
                print("Invalid input. Angular velocity must be a number.")

        # Create and populate Twist message
        turtle_vel = Twist()
        turtle_vel.linear.x = linear_x
        turtle_vel.angular.z = angular_z

        # Publish to the selected turtle
        if turtle_name == "turtle1":
            pub_turtle1.publish(turtle_vel)
        elif turtle_name == "turtle2":
            pub_turtle2.publish(turtle_vel)

        # Move the turtle for 1 second
        rospy.sleep(1.0)

        # Stop the turtle
        turtle_vel.linear.x = 0.0
        turtle_vel.angular.z = 0.0
        if turtle_name == "turtle1":
            pub_turtle1.publish(turtle_vel)
        elif turtle_name == "turtle2":
            pub_turtle2.publish(turtle_vel)

        rate.sleep()


if __name__ == "__main__":
    main()
