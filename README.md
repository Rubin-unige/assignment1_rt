# Research Track I - First Assignment
This repository contains the assignment work for the **Research Track** course, completed by:  
**Rubin Khadka Chhetri**  
**ID: 6558048**

## Table of Contents
- [Introduction](#introduction)
- [Node Details](#node-details)
  - [User Interface Node](#1-user-interface-node-user_interface)
  - [Distance Monitor Node](#2-distance-monitor-node-distance_monitor)
- [Repository Structure](#repository-structure)
- [Getting Started (Read Before Action)](#getting-started-read-before-action)
  - [Prerequisites](#prerequisites)
  - [Setup](#setup)
- [Launching Simulation and Nodes](#launching-simulation-and-nodes)
  - [Running C++ version](#running-the-c-version)
  - [Running Python version](#running-the-python-version)
  - [Alternative Configurations](#alternative-configurations)
- [Implementation Details](#implementation-details)
  - [User Interface Node](#user-interface-node)
  - [Distance Monitor Node](#distance-monitor-node)

## Introduction
This repository implements a ROS package containing two main nodes: 

-  **User Interface node**
-  **Distance Monitor node**

These nodes work together within the **`turtlesim`** simulation environment to create a simple, interactive system for controlling and monitoring two turtles.

**Note**:
  - This assignment was completed using both **Python** and **C++**. 
  - Custom message is included in this project, which is not required. Good for further development 

## Node Details
### 1. User Interface Node (user_interface)

This node is responsible for handling user input and controlling the movements of two turtles (`turtle1` and `turtle2`) in the simulator. Its key functions include:
-  Spawns a second turtle (`turtle2`) in the simulation environment.
-  Prompts the user to:
   -  Select which turtle to control (either `turtle1` or `turtle2`).
   -  Set the selected turtle's linear and angular velocities.
-  Sends movement commands to the selected turtle, causing it to move for one second. After the movement, the turtle stops, and the interface is ready to accept the next command.

### 2. Distance Monitor Node (distance_monitor)

This node ensures that the turtles maintain safe distances from each other and stay within the boundaries of the simulation environment. This node continuously calculates and monitors turtle positions. Its key features are:
-  Continuously calculates the distance between `turtle1` and `turtle2` and publishes this information on a dedicated ROS topic for monitoring.
-  Automatically stops a turtle if it approaches the other turtle.
-  Stops a turtle if it's position is too close to the boundaries.

## Repository Structure
The root of this repository is the package folder, which contains all necessary files and scripts for running the assignment nodes. When cloning the repository for the first time, place it directly in the `src` folder of your ROS workspace.

### Folder and File Overview
- **`/msg`**: Contains custom message definitions.
  - `turtle_distance.msg`: Custom message file for distance monitoring and boundary status.

- **`/scripts`**: Contains Python scripts used for the nodes in this project. 
  - `user_interface.py`: Python version of user interface node.
  - `distance_monitor.py`: Python version of distance monitor node.

- **`/src`**: Contains C++ source files.
  - `user_interface.cpp`: C++ version of the user interface node.
  - `distance_monitor.cpp`: C++ version of the distance monitor node.

- **`/CMakeLists.txt`**: Specifies the package build rules.

- **`/package.xml`**: Lists dependencies and package metadata.

- **`/README.md`**: Read me file.

## Getting Started (Read Before Action)

### Prerequisites
Before proceeding, make sure that **`ROS Noetic`** is installed on your system.<br>
If you haven’t set up ROS yet, check this official guide to install ROS: <br>
(https://wiki.ros.org/noetic/Installation/Ubuntu) <br>

Additionally, you’ll need **`Python 3`** and **`turtlesim`** package to run this project. Ensure they are installed on your system. If not, you can install them by running:
```bash
sudo apt-get update
sudo apt-get install ros-noetic-turtlesim
sudo apt-get install python3
```
After installation, you can proceed to cloning the repository.

### Setup 
#### 1. Set up your ROS workspace

Create a new workspace (or use an existing one) and navigate to its `src` directory:
```bash
mkdir -p ~/my_new_ws/src
cd ~/my_new_ws/src
```
#### 2. Clone this repository

Clone the assignment repository into your workspace’s `src` folder:
```bash
git clone https://github.com/Rubin-unige/assignment1_rt.git
```
#### 3. Add the Workspace to Your ROS Environment

To ensure that your workspace is sourced automatically every time you start a new terminal session, add it to your `.bashrc` file:
```bash
echo "source ~/my_new_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### 4. Build the Package

Navigate to the root of your workspace and build the package using `catkin_make`:
```bash
cd ~/my_new_ws
catkin_make
```
After building, your workspace will be ready to launch the nodes in the package.

## Launching Simulation and Nodes

#### 1. Start the ROS Master

Before running any ROS nodes, make sure the ROS Master is up and running. Open a terminal and start `roscore`:
```bash
roscore
```
#### 2. Run the Turtlesim Node

Next, start the `Turtlesim` node in a new terminal to launch the simulation environment:
```bash
rosrun turtlesim turtlesim_node
```
This will open the Turtlesim window where the turtles (`turtle1` and `turtle2`) will appear.

#### 3. Run the User Interface and Distance Monitor Nodes

At this point, you can proceed to run either the **C++** or **Python** version of the `User Interface` and `Distance Monitor` nodes, depending on which implementation you want to use. The nodes can be run individually or in combination, offering flexibility in how you choose to execute them.

#### Running the C++ Version
---
To run the C++ nodes, follow these steps:
- Make sure that the `roscore` and `turtlesim` nodes are running.

- In a new terminal, run the **C++ User Interface Node**:
```bash
rosrun assignment1_rt user_interface
```
- In another terminal, run the **C++ Distance Monitor Node**:
```bash
rosrun assignment1_rt distance_monitor
```
This will start both the **C++ user interface** for controlling the turtles and the **distance monitor** to track their movements.

#### Running the Python Version
---
To run the Python nodes, follow these steps:
- Make sure that the `roscore` and `turtlesim` nodes are running.

- Make the **Python scripts executable**

Before running the Python scripts, you need to ensure they are executable. Run the following command for each Python script (`user_interface.py` and `distance_monitor.py`):
``` bash
chmod +x ~/my_new_ws/src/assignment1_rt/scripts/user_interface.py
chmod +x ~/my_new_ws/src/assignment1_rt/scripts/distance_monitor.py
```

- After making the scripts executable, run the **Python User Interface Node** in the same terminal:
```bash
rosrun assignment1_rt user_interface.py
```
- Open a new terminal and run the **Python Distance Monitor Node**:
```bash
rosrun assignment1_rt distance_monitor.py
```
This will start both the **Python user interface** for controlling the turtles and the **distance monitor** to track their movements.

#### Alternative Configurations
---

The program is flexible, allowing you to mix and match the **C++** and **Python** nodes based on your preference. For example, you can run the `C++ User Interface` node while running the `Python Distance Monitor` node, or vice versa. This allows you to run the system in the configuration that best suits your workflow and testing needs.

#### 4. Stopping the nodes

To stop the nodes, simply press `Ctrl+C` in the terminal where each node is running (`User Interface`, `Distance Monitor`, `Turtlesim`, or `roscore`). This will terminate the nodes and stop the simulation.

## Implementation Details

### User Interface Node

The structure of the `user_interface` node is similar in both **C++** and **Python**. The logic for handling user inputs, setting velocities, and publishing commands is nearly identical in both languages. Since the logic for both versions is fundamentally the same, I will explain the details using the **C++** version as an example.

However, there was one key difference when running the node in **Python**: if the user closed the node and opened it again, `turtle2` would already exist in the simulation, causing a conflict. This issue did not occur in the **C++** implementation, where if `turtle2` already existed, the node would simply display a message saying "turtle2 already exists" and take the position from the already existing turtle.

In the **Python** version, when the node is restarted, attempting to spawn `turtle2` again would cause the node to crash because `turtle2` already existed in the simulation. To address this, additional checks were implemented to ensure that `turtle2` is only spawned if it doesn't already exist. This solution is explained below in separate section.

### 1. Spawning Turtle2

The `user_interface` node automatically spawns a second turtle, `turtle2`, in the simulation when the program starts. This is accomplished using the `/spawn` service provided by `turtlesim`, which allows for creating a new turtle at a specified position and orientation in the simulation environment.

In this implementation:

- `turtle2` is spawned at coordinates **(5.0, 2.0)** with an orientation of **0.0** radians, positioning it to face directly to the right.
- The `/spawn` service is called during the initialization phase of the node, ensuring that `turtle2` is added automatically without requiring any user input.

Below is the code that sets up the spawn request:
```cpp
// Initialise service clients
ros::ServiceClient client_spawn = nh.serviceClient <turtlesim::Spawn> ("/spawn");
turtlesim::Spawn spawn_srv;
// Spawn Turtle
spawn_srv.request.x = 5.0;
spawn_srv.request.y = 2.0;
spawn_srv.request.theta = 0.0;
spawn_srv.request.name = "turtle2";
client_spawn.call(spawn_srv);
```
### 2. User Interface

The user interface of the `user_interface` node allows the user to control either `turtle1` or `turtle2` by setting their velocities.

#### 1. Selecting the Turtle

The user is prompted to select a turtle (either `turtle1` or `turtle2`). If an invalid input is provided, the program will re-prompt the user until a valid turtle name is entered

```cpp
std::cout << "Enter the turtle you want to control (turtle1 or turtle2): ";
std::cin >> turtle_name;
if (turtle_name != "turtle1" && turtle_name != "turtle2") {
  std::cout << "Invalid turtle name. Please enter 'turtle1' or 'turtle2'.\n";
  continue;
}
```
#### 2. Setting Velocities

After selecting a turtle, the user is asked to enter the linear and angular velocities. Error handling ensures only valid numeric inputs are accepted.

  - Linear Velocity (x):<br> 
  The user is prompted for the linear velocity. If the input is invalid, the program clears the error state and asks the user to re-enter a valid value.
  ```cpp
  std::cout << "Enter the linear velocity (x): ";
  while (!(std::cin >> linear_x)) {
    std::cout << "Invalid input. Please enter a valid number for linear velocity: ";
    std::cin.clear();  
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  ```
  - Angular Velocity (z):<br>
  The same process is repeated for the angular velocity input.
  ```cpp
  std::cout << "Enter the angular velocity (z): ";
  while (!(std::cin >> angular_z)) {
    std::cout << "Invalid input. Please enter a valid number for angular velocity: ";
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  ```

#### Error Handling Issue

During the initial implementation of the node, I faced an issue with invalid inputs for the velocities. If the user entered a non-numeric value, the program would crash or behave unexpectedly. To resolve this, I added error handling that clears the input buffer and prompts the user to re-enter valid values for both the linear and angular velocities. 

### 3. Publishing User Input

Once the user has selected the turtle and entered the linear and angular velocities, the `user_interface` node publishes these commands to the respective turtle's velocity topic. 

The following steps are taken to publish the user inputs:

#### 1. Turtle Selection
  After the user selects the turtle, the node checks which turtle was selected and publishes the corresponding velocity commands to the respective topic.
  - `turtle1` uses the topic `/turtle1/cmd_vel`.
  - `turtle2` uses the topic `/turtle2/cmd_vel`.
```cpp
ros::Publisher pub_turtle1 = nh.advertise <geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
ros::Publisher pub_turtle2 = nh.advertise <geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
```
#### 2. Publishing the Velocity Command
  A `geometry_msgs::Twist` message is created, which holds the linear and angular velocities. This message is then published to the appropriate topic using the publisher.

Below is the code for publishing the velocities:
```cpp
geometry_msgs::Twist turtle_vel;
turtle_vel.linear.x = linear_x;
turtle_vel.angular.z = angular_z;
if (turtle_name == "turtle1")
{
  pub_turtle1.publish(turtle_vel);
  ros::Duration(1.0).sleep();
  turtle_vel.linear.x = 0.0;
  turtle_vel.angular.z = 0.0;
  pub_turtle1.publish(turtle_vel);
}
else if (turtle_name == "turtle2") {
  pub_turtle2.publish(turtle_vel);  
  ros::Duration(1.0).sleep();    
  turtle_vel.linear.x = 0;         
  turtle_vel.angular.z = 0;
  pub_turtle2.publish(turtle_vel); 
} 
```
#### Explanation of the Code
  - `geometry_msgs::Twist turtle_vel;`: This message holds the velocity commands for the turtle. The `linear.x` field holds the linear velocity, and `angular.z` holds the angular velocity.
  - `turtle_vel.linear.x = linear_x;` and `turtle_vel.angular.z = angular_z;`: These lines set the user-defined velocities for linear and angular movement.
  - `pub_turtle1.publish(turtle_vel);` and `pub_turtle2.publish(turtle_vel);`: Depending on the selected turtle, the program publishes the velocity message to the appropriate topic (`/turtle1/cmd_vel` or `/turtle2/cmd_vel`).
  - `ros::Duration(1.0).sleep();`: This command pauses the program for 1 second, allowing the turtle to move. 
  `ros::Duration(1.0).sleep()` is used instead of `sleep(1.0)` because it is specifically designed for ROS nodes, ensuring proper synchronization with the ROS event loop and preventing any timing issues.

#### 3. Stopping the Turtle
After 1 second, the velocities are set to 0 (both linear and angular) to stop the turtle, and the stop command is published to the respective turtle.

### 4. Python `turtle2` already exist issue
As discussed earlier, when restarting the `user_interface` node in **Python**, the node crashed because `turtle2` already existed in the simulation. This issue did not occur in the **C++** version.

To address this problem in **Python**, an additional check was implemented to ensure that `turtle2` already exists in the simulation before attempting to spawn it. This check uses the `/turtle2/pose` topic, which is only active when `turtle2` is present. The solution involves the following steps:

- Check if `turtle2` Exists:
  - Subscribe to the `/turtle2/pose` topic, which publishes the position and orientation of `turtle2`.
  - If a message is received within a 1-second timeout, it confirms that `turtle2` already exists in the simulation, and no further action is needed.
- Spawn `turtle2` if Not Found:
  - If no message is received within the timeout, the node assumes that `turtle2` does not exist.
  - The node then calls the `/spawn` service to create `turtle2` at the coordinates **(5.0, 2.0)** with an orientation of **0.0**.

Below is the code that sets up this check:
```Python
def check_if_turtle2_exists():
  ## Checks if turtle2 exists by subscribing to /turtle2/pose.
  turtle2_exists = False
  def pose_callback(msg):
      nonlocal turtle2_exists
      turtle2_exists = True  # Message received; turtle2 exists
  # Subscribe to /turtle2/pose topic
  rospy.Subscriber('/turtle2/pose', Pose, pose_callback)
  # Short delay , wait message
  timeout_time = rospy.Time.now() + rospy.Duration(1.0)  # 1-second timeout
  while rospy.Time.now() < timeout_time and not rospy.is_shutdown():
      rospy.sleep(0.1)  # Sleep in small increments
  return turtle2_exists
```

---
### Distance Monitor Node

Similar to  `user interface` node, the `Distance Monitor` Node shares a similar structure in both **Python** and **C++**, with the core logic remaining consistent across both implementations. I will use the **C++** version as the primary example to explain the implementation.

### 1. Set boundary conditions
In `turtlesim`, the window has predefined coordinates where the turtles' positions are constrained. The boundary conditions are essential to keep the turtles within the window and avoid collisions. The following conditions are defined:

  - **boundary_limit**: This is the minimum allowed position (in both the x and y directions). If the turtle's position gets close to this limit, it is considered near the boundary. Here it is set to 1.0.

  - **max_limit**: This is the maximum allowed position for a turtle. If the turtle exceeds this limit, it is considered out of bounds, and corrective actions should be taken. Here it is set to 10.0.

  - **distance_threshold**: This defines the minimum safe distance between the two turtles. If the turtles get closer than this distance, they are considered to be too close and need to be stopped to avoid collision. Here it is set to 2.0.
```cpp
// Global variables to store turtle positions
float turtle1_x, turtle1_y, turtle2_x, turtle2_y;
const float distance_threshold = 2.0;
const float boundary_limit = 1.0;
const float max_limit = 10.0;
```
### 2. Calculate the distance between two turtles

To calculate the distance between two turtles, the **Euclidean distance** formula is used. This formula determines the straight-line distance between two points in a 2D space based on their **x** and **y** coordinates. The formula is:

$$
distance = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}
$$

Where:
- $\( (x_1, y_1) \)$ are the coordinates of turtle1,
- $\( (x_2, y_2) \)$ are the coordinates of turtle2.

This calculation helps in monitoring whether the turtles are too close to each other, based on the **distance_threshold**.
```cpp
// Calculate the distance between turtles
float distance = sqrt(pow(turtle2_x - turtle1_x, 2) + pow(turtle2_y - turtle1_y, 2));
```
Now, to perform this calculation, we need the **x** and **y** values for both turtles. These values are retrieved by subscribing to the `/turtle1/pose` and `/turtle2/pose` topics. Each turtle continuously publishes its position in the **Pose** message format, which includes the **x** and **y** coordinates.
```cpp
// Subscribers for turtle positions
ros::Subscriber sub_turtle1 = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
ros::Subscriber sub_turtle2 = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);
```
**turtle1PoseCallback** and **turtle2PoseCallback** are callback functions that update the global position variables (turtle1_x, turtle1_y, turtle2_x, turtle2_y) whenever new **Pose** messages are received from each turtle.

```cpp
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
  turtle1_x = msg->x;
  turtle1_y = msg->y;
  ROS_INFO("Turtle1 updated position: (%.2f, %.2f)", turtle1_x, turtle1_y);
}
```
### 3. Check if the turtle are too close to each other
After calculating the distance, we use the **distance_threshold** to check if the turtles are too close. If the distance is less than the threshold, a flag is set to indicate that the turtles need attention. The following check updates the flag:
```cpp
// Check if turtles are too close
bool is_too_close = (distance < distance_threshold);
```
This flag can then be used to trigger any necessary actions.

### 4. Check if the turtle are near boundary
To monitor if the turtles are approaching the boundary, we use a boolean flag. This flag is set by checking whether either of the turtles' coordinates are nearing or exceeding the predefined boundary limits. We perform this check using the `is_near_boundary()` function, which compares both the x and y positions of the turtles against the boundary limits (`boundary_limit` and `max_limit`). If any coordinate is outside the allowed range, the function returns `true`, indicating that the turtle is near the boundary.
```cpp
// Check if turtles are near boundaries
bool turtle1_near_boundary = is_near_boundary(turtle1_x, turtle1_y);
bool turtle2_near_boundary = is_near_boundary(turtle2_x, turtle2_y);
```
The `is_near_boundary` function is defined as:
```cpp
bool is_near_boundary(float x, float y)
{
  return (x < boundary_limit || x > max_limit || y < boundary_limit || y > max_limit);
}
```
If any of the flags (distance or boundary) are triggered, necessary actions are performed.

### 5. Perform necessary action
If the turtles are too close to each other or near the boundary, they need to be stopped. This is done by publishing zero velocity commands to each turtle.
```cpp
  // If turtles are too close or near boundary, stop them
  if (is_too_close || turtle1_near_boundary || turtle2_near_boundary)
  {
    if (is_too_close){
        stopTurtle(pub_turtle1);  // Stop turtle1
        stopTurtle(pub_turtle2);  // Stop turtle2
    }
    if (turtle1_near_boundary){
        stopTurtle(pub_turtle1);  // Stop turtle1
    }
    if (turtle2_near_boundary){
        stopTurtle(pub_turtle2);  // Stop turtle2
    }
  }
```
The `stopTurtle()` function stops a turtle by publishing a `geometry_msgs::Twist` message with zero velocity:
```cpp
void stopTurtle(ros::Publisher &pub){
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    pub.publish(stop_msg);
}
```
This ensures that turtles stop moving when they're too close or near the boundaries.

### 6. Check for overshoot and handle this issue

## Summary