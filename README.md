# moveit_UR5_test
Some examples which use `moveit!` to manipulate UR5 (robot arm)

## How to run real ur5 model?
### 1. Create a workspace
```
mkdir -p ws_moveit/src
```

### 2. Install necessary softwares

* [universal_robots](http://wiki.ros.org/action/show/universal_robots?action=show&redirect=universal_robot)  
   ROS-Industrial support for Univesal Robots manipulators.

### 3. Configure hardware
>  To enable networking, use the UR’s teach-pendant to navigate to the Setup Robot -> Setup Network Menu (shown in the below attachment).  
**Note**: In order to test the communication between computer and robot arm, you can use this command:
```
ping IP_OF_THE_ROBOT
```

### 4. Describe the arm to ROS
#### 1). Install the package: [ROS_Industrial](http://wiki.ros.org/Industrial/Install) 
#### 2). Run the following launch files: 
```
roslaunch ur_description ur5_upload.launch
roslaunch ur_description test.launch
```
> You should see an rviz window showing the UR5 in a lying-down position, and a separate window where the joint values may be manually specified. Note that this is not a simulation, just a visualization of the arm model. To simulate UR5 or UR10, see ur_gazebo.

### 5. Making contact with UR5
#### 1). Install package [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver)
#### 2). Make the package 
```
catkin_make
```
#### 3). To bring up the real robot, run this command: 
```
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]
```
Note: the IP 10.10.10.60 is for the UR5e on the left. The IP for the UR5e on the right is 10.10.10.61

> Where ROBOT_IP_ADDRESS is your UR arm's IP and XX is '5' or '10' depending on your robot. The above launch file makes calls to both roscore and the launch file to the urXX_description so that ROS's parameter server has information on your robot arm. If you do not have your ur_description installed please do so via:
```
sudo apt install ros-<distro>-ur-description
```

### 6. Using Moveit! with hardware 
####　１). For setting up the MoveIt! nodes to allow motion planning run (assumes the connection is already established from section 4.3 above):
```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
```
#### 2). For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:
```
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```
> Note that as MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:
```
roslaunch ur_bringup ur5_bringup.launch limited:=true robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

### 7. Create your own ROS package
```
catkin_create_pkg [package name] ... 
```

### 8.　Install necessary dependency

```
cd ~/ws_moveit/src
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic
```
### 9. Make or build workspace 
```
cd ~/ws_moveit
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```
### 10. Add `setup.bash` 

```
source ~/ws_moveit/devel/setup.bash
# echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc
```

