## Project: Kinematics Pick & Place (Kuka KR210)

Nisheed Rama
18 Feb 2018
---

### Environment Setup

#### Prerequisites
##### Current Udacity Student
To setup you VM [click here](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/2919466f-aa2b-4424-b86a-98b0a53ce335/lessons/2cd33882-d29e-43e8-9ff7-10398c8b5351/concepts/undefined)

##### Others
You'll need to setup an environemnt with the following:
1. Ubuntu 16.04 LTS OS (https://www.ubuntu.com/download/desktop?cp=close)
1. ROS Kinetic 1.12.12 (http://wiki.ros.org/kinetic/Installation)
1. Gazebo 7.9 (http://gazebosim.org/download)
1. RVIZ version 1.12.15 (Qt version 5.5.1 & OGRE version 1.9.0) (http://wiki.ros.org/rviz)


###### Set up your ROS Workspace.
1. Download or clone the [project repository](https://github.com/nisheed75/RoboND-Kinematics-Project.git) into the ***src*** directory of your ROS Workspace. 
```sh
  cd ~/catkin_ws/src
  git clone https://github.com/nisheed75/RoboND-Kinematics-Project.git 
```
1. As this project uses custom Gazebo 3D models, we need to add the path through environment variable: 
```sh* As this project uses custom Gazebo 3D models, we need to add the path through environment variable: 
```sh
$ echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/kuka_arm/models" >> ~/.bashrc
```
1. Install missing ROS dependencies using the `rosdep` install command:
```sh
$ cd ~/catkin_ws/
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
1. Run catkin_make from within your workspace to build the project:
```sh
$ cd ~/catkin_ws/
$ catkin_make
```
1. Run the following shell commands to source the setup files:
```sh
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
1. Run the following shell commands to source the setup files:
```sh
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

1. For demo mode make sure the demo flag is set to `true` in `inverse_kinematics.launch` file under `~/catkin_ws/src/kuka_arm/launch/`


1. You can also control the spawn location of the target object in the shelf by modifying the spawn_location argument in `target_description.launch` file under `~/catkin_ws/src/kuka_arm/launch/`. 0-9 are valid values for spawn_location with 0 being random mode.

1. To run forward kinematics test us:
```sh
$ roslaunch kuka_arm forward_kinematics.launch
```

1. To run simulator use:
```sh
$ rosrun kuka_arm safe_spawner.sh
```

1. To run IK Server use:
```sh
$ rosrun kuka_arm IK_server.py 
```
$ echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/kuka_arm/models" >> ~/.bashrc
```

1. Install missing ROS dependencies using the `rosdep` install command:
```sh
$ cd ~/catkin_ws/
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

1. Run catkin_make from within your workspace to build the project:
```sh
$ cd ~/catkin_ws/
$ catkin_make
```

1. Run the following shell commands to source the setup files:
```sh
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

1. For demo mode make sure the demo flag is set to `true` in `inverse_kinematics.launch` file under `~/catkin_ws/src/kuka_arm/launch/`


1. You can also control the spawn location of the target object in the shelf by modifying the spawn_location argument in `target_description.launch` file under `~/catkin_ws/src/kuka_arm/launch/`. 0-9 are valid values for spawn_location with 0 being random mode.

1. To run forward kinematics test us:
```sh
$ roslaunch kuka_arm forward_kinematics.launch
```

1. To run simulator use:
```sh
$ rosrun kuka_arm safe_spawner.sh
```

1. To run IK Server use:
```sh
$ rosrun kuka_arm IK_server.py 
```


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


