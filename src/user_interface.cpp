
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

float linear_x, angular_z;

int main( int argc, char **argv){

    // Initialise ros 
    ros::init(argc, argv, "turtle_user_interface");
    ros::NodeHandle nh;

    // Initialise service client
    ros::ServiceClient client_spawn = nh.serviceClient <turtlesim::Spawn> ("/spawn");
    // Spawn turtle2
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 5.0;
    spawn_srv.request.y = 2.0;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle2";
    client_spawn.call(spawn_srv);

    // Initialize publishers
    ros::Publisher pub_turtle1 = nh.advertise <geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pub_turtle2 = nh.advertise <geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    std::string turtle_name;

    while (ros::ok()) {
        // User Input
        std::cout << "Enter the turtle you want to move (turtle1 or turtle2): ";
        std::cin >> turtle_name;
        // Ask for velocity
        std::cout << "Enter linear velocity x: ";
        std::cin >> linear_x;
        std::cout << "Enter angular velocity theta: ";
        std::cin >> angular_z;

        // Create the twist message
        geometry_msgs::Twist turtle_vel;  
        turtle_vel.linear.x = linear_x;   
        turtle_vel.angular.z = angular_z; 

        // Control turtle1
        if (turtle_name == "turtle1") {
            pub_turtle1.publish(turtle_vel); 
            ros::Duration(1.0).sleep();    
            turtle_vel.linear.x = 0;        
            turtle_vel.angular.z = 0;
            pub_turtle1.publish(turtle_vel);  

        // Control turtle2
        } else if (turtle_name == "turtle2") {
            pub_turtle2.publish(turtle_vel);  
            ros::Duration(1.0).sleep();    
            turtle_vel.linear.x = 0;      
            turtle_vel.angular.z = 0;
            pub_turtle2.publish(turtle_vel); 
        } 
        
        else {
            // Error if anything else is selected by user
            ROS_WARN("Invalid turtle name. Please choose 'turtle1' or 'turtle2'.");
        }

        ros::spinOnce(); 
    }

    return 0;

}