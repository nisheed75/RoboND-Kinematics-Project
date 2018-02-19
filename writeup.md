## Project: Kinematics Pick & Place (Kuka KR210)

Nisheed Rama
18 Feb 2018
---
[//]: # (Image References)

[image1]: https://github.com/nisheed75/RoboND-Kinematics-Project/blob/master/images/Kuka_KR210.jpg
[image2]: https://github.com/nisheed75/RoboND-Kinematics-Project/blob/master/images/KR210-simple.jpg
[image3]: https://github.com/nisheed75/RoboND-Kinematics-Project/blob/master/misc_images/eq2.png
[image4]: https://github.com/nisheed75/RoboND-Kinematics-Project/blob/master/misc_images/eq1.png


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



### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To get the DH parameter you need to look at the URDF file `kr210.urdf.xacro` and find the joints defintions, for example:
```xml
 <!-- joints -->
  <joint name="fixed_base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
```
Each joint definiton has an  **origin** tag that has the x, y ,z and the roll, yaw and pitch values:
```xml
<origin xyz="0 0 0" rpy="0 0 0"/>
```
The Kuka KR 210 arm has the following specification, I've drawn a schematic with the joints and link and plotted the X, Z axes in the diagram below. <br>  

![Schematic Kuka_KR210][image1]

Looking at the definiton in the  URDF file `kr210.urdf.xacro`  you can extract the parameters for the joint, links and gripper: <br>

| O | joint | parent | child | x | y | z | r | p | y |
|---|---|---|---|---|---|---|---|---|---|
|0 | fixed_base | base_footprint | base_link | 0 | 0 | 0 | 0 | 0 | 0 |
|1 | joint_1 | base_link | link_1 | 0 | 0 | 0.33 | 0 | 0 | 0 |
|2 | joint_2 | link_1 | link_2 | 0 .35| 0 | 0.42 | 0 | 0 | 0 |
|3 | joint_3 | link_2 | link_3 | 0 | 0 | 1.25 | 0 | 0 | 0 |
|4 | joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054 | 0 | 0 | 0 |
|5 | joint_5 | link_4 | link_5 | 0.54 | 0 | 0 | 0 | 0 | 0 |
|6 | joint_6 | link_5 | link_6 | 0.193 | 0 | 0 | 0 | 0 | 0 |
|7 | gripper | link_6 | gripper_link | 0.11 | 0 | 0 | 0 | 0 | 0 |
|. | **Total (m)** |  |  | **2.153** | 0 | **1.946** | 0 | 0 | 0 |

To simplify the translation I combine the last three joints (4, 5, and 6) at joint_5 since their axes intersect at a single point which represent the center of the robot spherical wrist, it will look like the diagram below:
![Schematic Kuka_KR210 - Simple][image2]
 
 #### Note that:

**Origin** O(i) = intersection between Xi and Zi axis

**Link Length:** a(i-1) = Zi-1 - Zi measured about the X(i-1) axis

**Link Offset:** d(i) = X(i-1) - X(i) measured about Z(i) axis

**Link Twist:** alpha(i-1) = angle from Z(i-1) to Z(i) measured about Xi-1 using right hand rule

**Joint Angle:** theta(i) = angle from X(i-1) to X(i) measured about Zi using right hand rule. all joint angles will be zero at initial Robot state in KR210 except joint 2 which has a -90 degree constant offset between X(1) and X(2).

**Gripper frame:** is the end point is the focal point for the Kinematic. It is displaced from Frame 6 by a translation along Z(6).

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The code of all my Kinematics can be found in the follwoing file `/kuka_arm/scripts/Kinematics.py`

Using the data and formulas described in question 1 you can derive the following table: <br>

|Links | i | alpha(i-1) | a(i-1) | d(i) | theta(i) |
|:---: | :---: | :---: | :---: | :---: | :---: |
|0->1 | 1 | 0 | 0 | 0.75 | q1 |
|1->2 | 2 | -90 | 0.35 | 0 | -90+q2 |
|2->3 | 3 | 0 |  | 1.25 | q3 |
|3->4 | 4 | -90 | -0.05 | 1.5 | q4 |
|4->5 | 5 | 90 | 0 | 0 | q5 |
|5->6 | 6 | -90 | 0 | 0 | q6 |
|6->7 | 7 | 0 | 0 | 0.303 | q7 |

To define the transformation matrix we can create the individual transforms between links, the DH convention uses four individual transforms define by the formulas below: 
![equation 1][image3]
![equation 2][image4]

Once you have the table above and the transfomation mateix above you can define the individual transformation bwtween link by substiting the variable in the matix with the values for each row in the table: e.g. the transformation from 6->7 will use the follwoing values amd substitutre this in the transformation matix. here is a code exmple of the individual transformation matices:
``` python
        T0_1 = self.transformation_matrix(q1, d1, a0, alpha0).subs(dh_table)
        T1_2 = self.transformation_matrix(q2, d2, a1, alpha1).subs(dh_table)
        T2_3 = self.transformation_matrix(q3, d3, a2, alpha2).subs(dh_table)
        T3_4 = self.transformation_matrix(q4, d4, a3, alpha3).subs(dh_table)
        T4_5 = self.transformation_matrix(q5, d5, a4, alpha4).subs(dh_table)
        T5_6 = self.transformation_matrix(q6, d6, a5, alpha5).subs(dh_table)
        T6_G = self.transformation_matrix(q7, d7, a6, alpha6).subs(dh_table)
```
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


