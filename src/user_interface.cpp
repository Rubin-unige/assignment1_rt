#include <ros/ros.h>
#include <std_msgs/String.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <unistd.h>
#include <limits>

float linear_x, angular_z;

int main(int argc, char **argv) {
    // Initialise ROS
    ros::init(argc, argv, "turtle_user_interface");
    ros::NodeHandle nh;

    // Initialise service clients
    ros::ServiceClient client_spawn = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_srv;

    // Spawn Turtle
    spawn_srv.request.x = 5.0;
    spawn_srv.request.y = 2.0;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle2";
    client_spawn.call(spawn_srv);

    // Initialize publishers
    ros::Publisher pub_turtle1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pub_turtle2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    std::string turtle_name;

    while (ros::ok()) {
        // User input
        std::cout << "Enter the turtle you want to control (turtle1 or turtle2): ";
        std::cin >> turtle_name;

        // Check if the turtle name is valid
        if (turtle_name != "turtle1" && turtle_name != "turtle2") {
            std::cout << "Invalid turtle name. Please enter 'turtle1' or 'turtle2'.\n";
            continue;
        }

        // Get the velocities
        std::cout << "Enter the linear velocity x (between -5 and 5): ";
        while (!(std::cin >> linear_x) || linear_x < -5 || linear_x > 5) { // Check range and input validity
            std::cout << "Invalid input. Please enter a linear velocity between -5 and 5: ";
            std::cin.clear(); // Clear error
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        std::cout << "Enter the angular velocity z (between -5 and 5): ";
        while (!(std::cin >> angular_z) || angular_z < -5 || angular_z > 5) { // Check range and input validity
            std::cout << "Invalid input. Please enter an angular velocity between -5 and 5: ";
            std::cin.clear(); // Clear error
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        // Create geometry msgs to move the turtle
        geometry_msgs::Twist turtle_vel;
        turtle_vel.linear.x = linear_x;
        turtle_vel.angular.z = angular_z;

        // Turtle1
        if (turtle_name == "turtle1") {
            pub_turtle1.publish(turtle_vel);
            ros::Duration(1.0).sleep();
            turtle_vel.linear.x = 0.0;
            turtle_vel.angular.z = 0.0;
            pub_turtle1.publish(turtle_vel);
        }
        // Turtle2
        else if (turtle_name == "turtle2") {
            pub_turtle2.publish(turtle_vel);
            ros::Duration(1.0).sleep();
            turtle_vel.linear.x = 0;
            turtle_vel.angular.z = 0;
            pub_turtle2.publish(turtle_vel);
        }

        ros::spinOnce();
    }

    ros::shutdown();
    
    return 0;
}
