#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "assignment1_rt/turtle_distance.h"
#include <cmath>

// Global variables to store turtle positions
float turtle1_x, turtle1_y, turtle2_x, turtle2_y;
const float distance_threshold = 2.0;
const float boundary_limit = 1.0;
const float max_limit = 10.0;

// Function declarations
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg);
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg);
bool is_near_boundary(float x, float y);

// Publisher for both turtles' velocity commands
ros::Publisher pub_turtle1, pub_turtle2;

// Function to stop the turtles by publishing zero velocity
void stopTurtle(ros::Publisher &pub)
{
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    pub.publish(stop_msg);
}

int main(int argc, char **argv)
{
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

    ros::Rate rate(5);  
    
    while (ros::ok())
    {
        ros::spinOnce();  // Process incoming messages

        // Calculate the distance between turtles
        float distance = sqrt(pow(turtle2_x - turtle1_x, 2) + pow(turtle2_y - turtle1_y, 2));

        // Check if turtles are too close
        bool is_too_close = (distance < distance_threshold);

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
                ROS_WARN("Turtles are too close, stopping them!");
                stopTurtle(pub_turtle1);  // Stop turtle1
                stopTurtle(pub_turtle2);  // Stop turtle2
            }

            if (turtle1_near_boundary)
            {
                ROS_WARN("Turtle1 is near boundary, stopping it!");
                stopTurtle(pub_turtle1);  // Stop turtle1
            }

            if (turtle2_near_boundary)
            {
                ROS_WARN("Turtle2 is near boundary, stopping it!");
                stopTurtle(pub_turtle2);  // Stop turtle2
            }
        }

        rate.sleep();
    }

    return 0;
}

void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_x = msg->x;
    turtle1_y = msg->y;
    ROS_INFO("Turtle1 updated position: (%.2f, %.2f)", turtle1_x, turtle1_y);
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_x = msg->x;
    turtle2_y = msg->y;
    ROS_INFO("Turtle2 updated position: (%.2f, %.2f)", turtle2_x, turtle2_y);
}

bool is_near_boundary(float x, float y)
{
    return (x < boundary_limit || x > max_limit || y < boundary_limit || y > max_limit);
}