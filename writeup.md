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
| Step | Action | Command |
|---|---|---|
| 1. | Download or clone the [project repository](https://github.com/nisheed75/RoboND-Kinematics-Project.git) into the ***src*** directory of your ROS Workspace. |  ` cd ~/catkin_ws/src ` <br>  ` git clone https://github.com/nisheed75/RoboND-Kinematics-Project.git ` |
| 2. | As this project uses custom Gazebo 3D models, we need to add the path through environment variable: | `$ echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/kuka_arm/models" >> ~/.bashrc ` |
| 3. | Install missing ROS dependencies using the `rosdep` install command: |  ` $ cd ~/catkin_ws/ ` <br> ` $ rosdep install --from-paths src --ignore-src --  rosdistro=kinetic -y ` |
| 4. | Run catkin_make from within your workspace to build the project: | ` $ cd ~/catkin_ws/ <br> $ catkin_make ` |
| 5. | Run the following shell commands to source the setup files: | ` $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc ` |
| 6. | For demo mode make sure the demo flag is set to `true` in `inverse_kinematics.launch` file under | ` ~/catkin_ws/src/kuka_arm/launch/ ` |
| 7. | You can also control the spawn location of the target object in the shelf by modifying the spawn_location argument in `target_description.launch` file under `~/catkin_ws/src/kuka_arm/launch/`. 0-9 are valid values for spawn_location with 0 being random mode.| |
| 8. | To run forward kinematics test us: | `$ roslaunch kuka_arm forward_kinematics.launch `|
| 9. | To run simulator use: | ` $ rosrun kuka_arm safe_spawner.sh ` |
| 10. | To run IK Server use: | `$ rosrun kuka_arm IK_server.py <br> $ echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/kuka_arm/models" >> ~/.bashrc ` |
| 12. | Install missing ROS dependencies using the `rosdep` install command: | ` $ cd ~/catkin_ws/ <br> $ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y ` |
| 12. | Run catkin_make from within your workspace to build the project: | ` $ cd ~/catkin_ws/ <br> $ catkin_make ` |
| 13. | Run the following shell commands to source the setup files: | ` $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
` |
| 14. | For demo mode make sure the demo flag is set to `true` in `inverse_kinematics.launch` file under | `~/catkin_ws/src/kuka_arm/launch/` |
| 15. | You can also control the spawn location of the target object in the shelf by modifying the spawn_location argument in `target_description.launch` file under `~/catkin_ws/src/kuka_arm/launch/`. 0-9 are valid values for spawn_location with 0 being random mode. | |
| 15. | To run forward kinematics test us: | `$ roslaunch kuka_arm forward_kinematics.launch `|
| 16. | To run simulator use: | `$ rosrun kuka_arm safe_spawner.sh ` |
| 17. | To run IK Server use: | `$ rosrun kuka_arm IK_server.py ` |

[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png


### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | i | alpha(i-1) | a(i-1) | d(i) | theta(i) |
:---: | :---: | :---: | :---: | :---: | :---: |
0->1 | 1 | 0 | 0 | 0.75 | q1 |
1->2 | 2 | -90 | 0.35 | 0 | -90+q2 |
2->3 | 3 | 0 |  | 1.25 | q3 |
3->4 | 4 | -90 | -0.05 | 1.5 | q4 |
4->5 | 5 | 90 | 0 | 0 | q5 |
5->6 | 6 | -90 | 0 | 0 | q6 |
6->7 | 7 | 0 | 0 | 0.303 | q7 |


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]

