# Assignment2_rt_Part1: Robot Simulation Project

## Overview:

This repository contains the ROS1 package `assignment_2_2024`, which enables robot control and monitoring in a simulated environment. The project includes several nodes that handle robot navigation, goal setting, and distance/velocity computation.

### Features:

- **Action Client Node**: Sends target coordinates to the robot and waits for feedback on whether the robot has reached the target.
- **Distance and Velocity Node**: Monitors the robot's distance from the target and its velocity.
- **Target Service Node**: Provides the last target coordinates set by the user through a service.

### Prerequisites

1. **Python 3**: Ensure Python 3 is installed on your system.
2. **ROS1 (Noetic)**: Ensure you have ROS1 (Noetic) installed.
3. **Xterm**: Used to run the nodes in separate terminals.

To install Xterm:
   ```bash
   sudo apt-get install xterm
   ```
4. **ROS Workspace**: Ensure you have a ROS workspace set up:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   ```
---

## File structure:

assignment_2_2024 <br>
├── action <br>
│   └── Planning.action <br>
├── config <br>
│   ├── sim.rviz <br>
│   └── sim2.rviz <br>
├── launch <br>
│   └── assignment1.launch <br>
├── msg <br>
│   └── RobotPoseVelocity.msg <br>
├── srv <br>
│   ├── DistSpeed.srv <br>
│   └── LastTarget.srv <br>
├── scripts <br>
│   ├── Action_client.py <br>
│   ├── bug_as.py <br>
│   ├── Dist_Vel_node.py <br>
│   ├── go_to_point_service.py <br>
│   ├── Target_service.py <br>
│   └── wall_follow_service.py <br>
├── urdf <br>
│   ├── robot2_laser.gazebo <br>
│   └── robot2_laser.xacro <br>
├── world <br>
│   └── assignment.world <br>
├── CMakeLists.txt <br>
└── package.xml

---

## How to Get Started

Follow these steps to clone the package, build it, and run the robot simulation


### Steps to Get Started

1. **Clone the Repository**:
   - Clone the assignment_2_2024 package into your ROS workspace:
     ```bash
     cd ~/catkin_ws/src
     git clone https://github.com/Mohamedags/assignment_2_2024.git
     ```

2. **Build the Workspace**:
   - Build the workspace using `catkin_make`:
     ```bash
     cd ~/catkin_ws
     catkin_make
     ```

3. **Make the Scripts Executable**:
   - Change the permissions to make all the Python scripts executable:
     ```bash
     cd ~/catkin_ws/src/assignment_2_2024/scripts
     chmod +x *.py
     ```

4. **Launch the Simulation**:
   - Launch the whole simulation by running the provided launch file::
     ```bash
     roslaunch assignment_2_2024 assignment1.launch
     ```
### Launch File Details

The launch file will:

- Include the simulation environment (`sim_w1.launch`).
- Set initial target coordinates using parameters (`des_pos_x`, `des_pos_y`).
- start the `bug_as`, `go_to_point_service`, `wall_follow_service`, `Target_service` nodes in the main terminal.
- Automatically start the following nodes in separate Xterm windows:
    - **`Action_client.py`**: Sends goals and waits for feedback.
    - **`Dist_Vel_node.py`**: Monitors distance and velocity.

**Note**: The `launch-prefix="xterm -hold -e"` ensures that the nodes open in separate Xterm windows.

---
## Message and Service Files:

### **msg** Files:

1. **`RobotPoseVelocity.msg`**:
    - This message is used to convey the robot's pose and velocity information.
    - Fields:
      - `float64 x`: The x-coordinate of the robot's position.
      - `float64 y`: The y-coordinate of the robot's position.
      - `float64 vel_x`: The robot's linear velocity along the x-axis.
      - `float64 vel_z`: The robot's angular velocity around the z-axis.

---

### **srv** Files:

1. **`DistSpeed.srv`**:
    - This service is used to get the robot's distance from a target and its average speed.
    - Request: None
    - Response:
      - `float64 distance`: The calculated distance between the robot and the target.
      - `float64 average_speed`: The average speed of the robot.


2. **`LastTarget.srv`**:
    - This service is used to provide the last set target coordinates.
    - Request: None
    - Response:
      - `float64 target_x`: The x-coordinate of the last target.
      - `float64 target_y`: The y-coordinate of the last target.
    
---
    
### Example Workflow

1. **Set Target**: The Action Client Node sends the target coordinates (x, y) to the robot. The robot will move toward the target, and the node will display feedback like "Goal Reached" or "Goal Failed".
    
2. **Monitor Robot**: The Distance and Velocity Node will display the robot's current distance from the target and its average speed.

3. **Query Target**: The Target Service Node can be queried for the last target coordinates using the `/last_goal service`.

### Flowchart
![Project Diagram](images/Flowchart.png)
   
- Illustration of the Robot Movement: 
![Example GIF](images/illustration_rt2_part1.gif)

