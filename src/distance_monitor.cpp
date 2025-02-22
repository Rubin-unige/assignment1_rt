/**
 * \file distance_monitor.cpp
 * \brief This node monitors the distance between two turtles and checks if they are near boundaries.
 * \author Rubin Khadka Chhetri
 * \version 1.0.0
 * \date 22/02/2025
 *
 * Subscribes to:
 * - /turtle1/pose (turtlesim/Pose): The pose of turtle1.
 * - /turtle2/pose (turtlesim/Pose): The pose of turtle2.
 *
 * Publishes to:
 * - /turtle1/cmd_vel (geometry_msgs/Twist): Velocity commands for turtle1.
 * - /turtle2/cmd_vel (geometry_msgs/Twist): Velocity commands for turtle2.
 * - /turtle_distance_topic (assignment1_rt/turtle_distance): Custom message containing distance and boundary information.
 *
 * Description:
 * This node calculates the distance between turtle1 and turtle2 and checks if they are too close or near the boundaries.
 * If the turtles are too close or near the boundaries, the node stops them by publishing zero velocity commands.
 * It also publishes a custom message containing the distance and boundary status of the turtles.
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "assignment1_rt/turtle_distance.h"
#include <cmath>

// Global variables to store turtle positions
float turtle1_x, turtle1_y, turtle2_x, turtle2_y;
const float distance_threshold = 2.0; // Threshold for turtles being too close
const float boundary_limit = 1.0;     // Boundary limit for turtles
const float max_limit = 10.0;         // Maximum limit for turtles

// Function declarations
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg);
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg);
void stopTurtle(ros::Publisher &pub);
bool is_near_boundary(float x, float y);
bool check_if_overshot_boundary(float x, float y);

// Publisher for both turtles' velocity commands
ros::Publisher pub_turtle1, pub_turtle2;

/**
 * \brief Main function for the turtle_distance_monitor node.
 * \param argc Number of command-line arguments.
 * \param argv Command-line arguments.
 * \return 0 on successful completion.
 *
 * This function initializes the ROS node, subscribes to turtle poses, and monitors the distance between the turtles.
 * It publishes velocity commands to stop the turtles if they are too close or near the boundaries.
 */
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "turtle_distance_monitor");
    ros::NodeHandle nh;

    // Subscribers for turtle positions
    ros::Subscriber sub_turtle1 = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber sub_turtle2 = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);

    // Publishers for controlling turtle velocities
    pub_turtle1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pub_turtle2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    // Publisher for TurtleDistance (custom message)
    ros::Publisher pub_distance = nh.advertise<assignment1_rt::turtle_distance>("/turtle_distance_topic", 10);

    ros::Rate rate(10);  // Loop rate of 10 Hz

    while (ros::ok())
    {
        ros::spinOnce();  // Process incoming messages

        // Calculate the distance between turtles
        float distance = sqrt(pow(turtle2_x - turtle1_x, 2) + pow(turtle2_y - turtle1_y, 2));

        // Check if turtles are too close
        bool is_too_close = (distance <= distance_threshold);

        // Check if turtles are near boundaries
        bool turtle1_near_boundary = is_near_boundary(turtle1_x, turtle1_y);
        bool turtle2_near_boundary = is_near_boundary(turtle2_x, turtle2_y);

        // Publish the TurtleDistance custom message
        assignment1_rt::turtle_distance dist_msg;
        dist_msg.distance = distance;
        dist_msg.is_too_close = is_too_close;
        dist_msg.turtle1_near_bound = turtle1_near_boundary;
        dist_msg.turtle2_near_bound = turtle2_near_boundary;
        pub_distance.publish(dist_msg);

        // If turtles are too close or near boundary, stop them
        if (is_too_close || turtle1_near_boundary || turtle2_near_boundary)
        {
            if (is_too_close)
            {
                stopTurtle(pub_turtle1);  // Stop turtle1
                stopTurtle(pub_turtle2);  // Stop turtle2

                // Overshoot error handling, just sends a warning for now
                float distance = sqrt(pow(turtle2_x - turtle1_x, 2) + pow(turtle2_y - turtle1_y, 2));
                if (distance < distance_threshold){
                   ROS_WARN("Turtles are really too close !!!");
                }
            }

            if (turtle1_near_boundary){
                stopTurtle(pub_turtle1);  // Stop turtle1
            }

            if (turtle2_near_boundary){
                stopTurtle(pub_turtle2);  // Stop turtle2
            }
        }

        // After stopping the turtles, check if they have overshot the boundary
        bool turtle1_overshot = check_if_overshot_boundary(turtle1_x, turtle1_y);
        bool turtle2_overshot = check_if_overshot_boundary(turtle2_x, turtle2_y);
        
        // Warning that the turtles have overshot the boundary condition
        if (turtle1_overshot){
            ROS_WARN("Turtle1 is over the boundary after stopping!");
        }

        if (turtle2_overshot){
            ROS_WARN("Turtle2 is over the boundary after stopping!"); 
        }

        rate.sleep();
    }
    return 0;
}

/**
 * \brief Callback function for turtle1's pose.
 * \param msg The message containing turtle1's pose.
 */
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_x = msg->x;
    turtle1_y = msg->y;
}

/**
 * \brief Callback function for turtle2's pose.
 * \param msg The message containing turtle2's pose.
 */
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_x = msg->x;
    turtle2_y = msg->y;
}

/**
 * \brief Check if a turtle is near the boundary.
 * \param x The x-coordinate of the turtle.
 * \param y The y-coordinate of the turtle.
 * \return True if the turtle is near the boundary, false otherwise.
 */
bool is_near_boundary(float x, float y){
    return (x <= boundary_limit || x >= max_limit || y <= boundary_limit || y >= max_limit);
}

/**
 * \brief Check if a turtle has overshot the boundary after stopping.
 * \param x The x-coordinate of the turtle.
 * \param y The y-coordinate of the turtle.
 * \return True if the turtle has overshot the boundary, false otherwise.
 */
bool check_if_overshot_boundary(float x, float y){
    if (x > max_limit || x < boundary_limit  || y > max_limit || y < boundary_limit ){
        return true;
    }
    return false;
} 

/**
 * \brief Stop a turtle by publishing zero velocity.
 * \param pub The publisher for the turtle's velocity commands.
 */
void stopTurtle(ros::Publisher &pub){
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    pub.publish(stop_msg);
}