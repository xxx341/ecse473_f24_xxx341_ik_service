# Inverse Kinematics Service for UR10 Robot

This ROS package implements an inverse kinematics (IK) service for the UR10 robot. It calculates joint configurations required to achieve a specified end-effector pose. The package includes a service node, a client node, and custom message/service definitions.

---

## Package Overview

1. **Nodes**

   - **`ik_service`**:
     - Service node that calculates IK solutions.
     - Provides the **`/pose_ik`** service.
   - **`ik_client`**:
     - Client node that sends a pose request to the **`/pose_ik`** service and logs the response.

2. Custom Definitions

   - Service: **`PoseIK.srv`**
     - Request:
       - **`part_pose`** (geometry_msgs/Pose): The desired end-effector pose.
     - Response:
       - **`num_sols`** (int32): Number of IK solutions found.
       - **`joint_solutions`** (array of **`JointSolutions`**): Valid joint configurations.
   - Message: **`JointSolutions.msg`**
     - joint_angles (float64[6]): Array of joint angles.

---

## Package Contents

### Directory Structure
ik_service/
├── launch/
│   └── ik_service.launch
├── src/
│   ├── ik_service.cpp
│   └── ik_client.cpp
├── srv/
│   └── PoseIK.srv
├── include
├── CMakeLists.txt
├── package.xml
└── README.md

---

## Getting Started

### Prerequisites
Ensure you have:
  - ROS Noetic installed on Ubuntu 20.04.
  - Necessary dependencies:
    ```bash
    sudo apt install ros-noetic-geometry-msgs ros-noetic-ur-kinematics
    ```
    
### Build the Package
1. Clone this repository into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone <repository_url>
   ```
2. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
3. Source the workspace:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```
   
---
 
## Usage
1. **Run the Service Node**:
   Start the **`ik_service`** node:
   ```bash
   rosrun ik_service ik_service
   ```
2. **Run the Client Node**
   In a new terminal, start the **`ik_client`** node:
   ```bash
   rosrun ik_service ik_client
   ```
3. **Call the Service Manually**
Use **`rosservice`** to test the service with a custom pose:
```bash
rosservice call /pose_ik "{part_pose: {position: {x: 0.2, y: 0.2, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```
Expected output:
```plaintext
num_sols: <number_of_solutions>
joint_solutions: 
  - 
    joint_angles: [<angle1>, <angle2>, ..., <angle6>]
```

---

## Testing and Validation
1. **Test Poses**
   - Reachable pose:
     ```bash
     rosservice call /pose_ik "{part_pose: {position: {x: 0.1, y: 0.1, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
     ```
   - Edge-of-workspace pose:
     ```
     rosservice call /pose_ik "{part_pose: {position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
     ```
2. **Launch Both Nodes**
Simplify testing by running both nodes with a launch file:
   1. Create a launch file:
      ```xml
      <launch>
        <node name="ik_service" pkg="ik_service" type="ik_service" output="screen" />
        <node name="ik_client" pkg="ik_service" type="ik_client" output="screen" />
      </launch>
      ```
   2. Run the launch file:
      ```bash
      roslaunch ik_service ik_service.launch
      ```

---

## Troubleshooting
1. No solutions found (**`num_sols: 0`**)
   - Ensure the pose is reachable within the UR10's workspace.
   - Try adjusting the q6_des parameter in the IK solver for better precision.
2. Nodes not found
   - Ensure your package is built and sourced:
     ```bash
     cd ~/catkin_ws
     catkin_make
     source ~/catkin_ws/devel/setup.bash
     ```
3. ROS master issues
   - Ensure roscore is running in a separate terminal:
     ```bash
     roscore
     ```
