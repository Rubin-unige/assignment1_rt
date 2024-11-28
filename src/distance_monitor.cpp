#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "assignment1_rt/turtle_distance.h"
#include <cmath>

// Global variables to store turtle positions
float turtle1_x, turtle1_y, turtle2_x, turtle2_y, turtle2_theta, turtle1_theta;
float prev_turtle1_x, prev_turtle1_y, prev_turtle2_x, prev_turtle2_y;
const float distance_threshold = 1.0;
const float boundary_limit = 1.0;
const float max_limit = 10.0;

// Function declarations
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg);
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg);
bool is_near_boundary(float x, float y);
bool check_if_overshot_boundary(float x, float y);
void reposition_turtle(ros::Publisher &pub, float x, float y, float prev_y, float theta);

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

    ros::Rate rate(10);  

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

                // overshoot error handling 
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
        
        // TODO: handle overshoot
        if (turtle1_overshot){
            ROS_WARN("Turtle1 is over the boundary after stopping!");
            reposition_turtle(pub_turtle1, turtle1_x, turtle1_y, prev_turtle1_y, turtle1_theta);
        }

        if (turtle2_overshot){
            ROS_WARN("Turtle2 is over the boundary after stopping!"); 
            reposition_turtle(pub_turtle2, turtle2_x, turtle2_y, prev_turtle2_y, turtle2_theta);
        }

        // Update previous positions at the end of each loop
        prev_turtle1_x = turtle1_x;
        prev_turtle1_y = turtle1_y;
        prev_turtle2_x = turtle2_x;
        prev_turtle2_y = turtle2_y;

        rate.sleep();
    }
    return 0;
}

void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_x = msg->x;
    turtle1_y = msg->y;
    turtle1_theta = msg->theta;
    ROS_INFO("Turtle1 updated position: (%.2f, %.2f, %.2f)", turtle1_x, turtle1_y, turtle1_theta);
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_x = msg->x;
    turtle2_y = msg->y;
    turtle2_theta = msg->theta;
    ROS_INFO("Turtle2 updated position: (%.2f, %.2f, %.2f)", turtle2_x, turtle2_y, turtle2_theta);
}

bool is_near_boundary(float x, float y){
    return (x < boundary_limit || x > max_limit || y < boundary_limit || y > max_limit);
}

// Function to check if the turtle is over the boundary after stopping
bool check_if_overshot_boundary(float x, float y){
    if (x > max_limit || x < boundary_limit  || y > max_limit || y < boundary_limit ){
        return true;
    }
    return false;
} 

void reposition_turtle(ros::Publisher &pub, float x, float y, float prev_y, float theta) {
    geometry_msgs::Twist move_back_msg;

    // Track Y-axis movement direction (using current and previous y values)
    bool is_moving_up = (y > prev_y);   
    bool is_moving_down = (y < prev_y); 
    bool same_value = (y == prev_y);

    // **Handling Y-axis overshooting**
    if (y > max_limit) {
        if (theta >= 0) {
            move_back_msg.linear.x = -0.2; 
        } else if (theta < 0) {
            move_back_msg.linear.x = 0.2;  
        }
    } else if (y < boundary_limit) {
        if (theta > 0) {
            move_back_msg.linear.x = 0.2;   
        } else if (theta < 0) {
            move_back_msg.linear.x = -0.2;  
        }
    }

    if (x > max_limit) {
        if (theta > 0) {
            if (is_moving_up) {
                move_back_msg.linear.x = -0.2;
            } else if(is_moving_down){
                move_back_msg.linear.x = 0.2;
            }
        }
        if (theta == 0){
            move_back_msg.linear.x = -0.2;
        } else if (theta == -3.14){
            move_back_msg.linear.x = 0.2;
        }
    } else if (theta < 0){
        if (is_moving_up) {
                move_back_msg.linear.x = 0.2;
            } else if(is_moving_down){
                move_back_msg.linear.x = -0.2;
            } else if (same_value){
                move_back_msg.linear.x = 0.2;
            }
    } 

    if (x < boundary_limit) {
        if (theta >= 0) {
            if (is_moving_up) {
                move_back_msg.linear.x = -0.2;
            } else if (same_value){
                move_back_msg.linear.x = -0.2;
            } else if(is_moving_down){
                move_back_msg.linear.x = 0.2;
            }
        }
        if (theta == 0){
            move_back_msg.linear.x = 0.2;
        } else if (theta == -3.14){
            move_back_msg.linear.x = -0.2;
        }
    } else if (theta < 0){
        if (is_moving_up) {
                move_back_msg.linear.x = 0.2;
            } else if(is_moving_down){
                move_back_msg.linear.x = -0.2;
            } else if (same_value){
                move_back_msg.linear.x = 0.2;
            }
    } 
    
    // Publish the velocity to move the turtle back
    pub.publish(move_back_msg);

    // Stop the turtle after moving it back within the boundaries
    ros::Duration(0.5).sleep(); // Let the turtle move back for a brief moment
    stopTurtle(pub); // Stop the turtle after it moves back
}