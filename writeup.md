## Project: Kinematics Pick & Place (Kuka KR210)

Nisheed Rama
18 Feb 2018
---
[//]: # (Image References)

[image1]: /images/Kuka_KR210.jpg
[image2]: /images/KR210-simple.jpg
[image3]: /misc_images/eq2.png
[image4]: /misc_images/eq1.png
[image5]: /misc_images/urdf_coord.png
[image6]: /misc_images/gripper_frame.png
[image7]: /misc_images/eq_inv_kine.png
[image8]: /misc_images/arc_tan.png
[image9]: /misc_images/arch_tan.gif
[image10]: /misc_images/law_of_cos.png
[image11]: /misc_images/inv_orin_kine1.png
[image12]: /misc_images/inv_orin_kine2.png
[image13]: /images/9_1_on_floor.jpg
[image14]: /images/9_10.jpg
[image15]: /misc_images/homo-xform-2.png
[image16]: /misc_images/equations.png
[image17]: /misc_images/l21-l-inverse-kinematics-new-design-fixed.png
[image18]: /misc_images/3side_eq.png
[image19]: /misc_images/law_of_cos.png
[image20]: /images/triangle.png
[image21]: /misc_images/intrinsicVextrinsic.png
### Environment Setup

#### Prerequisites
##### Current Udacity Student
To setup you VM [click here](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/2919466f-aa2b-4424-b86a-98b0a53ce335/lessons/2cd33882-d29e-43e8-9ff7-10398c8b5351/concepts/undefined)

##### Others
You'll need to setup an environment with the following:
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

Looking at the definition in the  URDF file `kr210.urdf.xacro`  you can extract the parameters for the joint, links and gripper: <br>

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

To simplify the translation I combine the last three joints (4, 5, and 6) at joint_5 since their axes intersect at a single point which represents the center of the robot spherical wrist, it will look like the diagram below:
![Schematic Kuka_KR210 - Simple][image2]
 
 #### Note that:

**Origin** O(i) = intersection between Xi and Zi axis

**Link Length:** a(i-1) = Zi-1 - Zi measured about the X(i-1) axis

**Link Offset:** d(i) = X(i-1) - X(i) measured about Z(i) axis

**Link Twist:** alpha(i-1) = angle from Z(i-1) to Z(i) measured about Xi-1 using right hand rule

**Joint Angle:** theta(i) = angle from X(i-1) to X(i) measured about Zi using right hand rule. all joint angles will be zero at initial Robot state in KR210 except joint 2 which has a -90 degree constant offset between X(1) and X(2).

**Gripper frame:** is the end point is the focal point for the Kinematic. It is displaced from Frame 6 by a translation along Z(6).

#### 2. Using the DH parameter table, you derived earlier, create individual transformation matrices about each joint. Also, generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The code of all my Kinematics can be found in the following file `/kuka_arm/scripts/Kinematics.py`

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

To define the transformation matrix we can create the individual transforms between links, the DH convention uses four individual transforms defined by the formulas below: 
![equation 1][image3]
![equation 2][image4]

Once you have the table above and the transformation matrix above you can define the individual transformation between link by substituting the variable in the matrix with the values for each row in the table: e.g. the transformation from 6->7 will use the following values and substitute this in the transformation matrix. Here is a code example of the individual transformation matrices:
``` python
        T0_1 = self.transformation_matrix(q1, d1, a0, alpha0).subs(dh_table)
        T1_2 = self.transformation_matrix(q2, d2, a1, alpha1).subs(dh_table)
        T2_3 = self.transformation_matrix(q3, d3, a2, alpha2).subs(dh_table)
        T3_4 = self.transformation_matrix(q4, d4, a3, alpha3).subs(dh_table)
        T4_5 = self.transformation_matrix(q5, d5, a4, alpha4).subs(dh_table)
        T5_6 = self.transformation_matrix(q6, d6, a5, alpha5).subs(dh_table)
        T6_G = self.transformation_matrix(q7, d7, a6, alpha6).subs(dh_table)
```
The transformation matrices for each joint are thus:

Joint 1 `T0_1`:
```
[[cos(q1), -sin(q1), 0, 0],
[sin(q1), cos(q1), 0, 0],
[0, 0, 1, 0.750000000000000],
[0, 0, 0, 1]]
```
Joint 2 `T1_2`:
```
[[cos(q2 - 0.5*pi), -sin(q2 - 0.5*pi), 0, 0.350000000000000],
[0, 0, 1, 0],
[-sin(q2 - 0.5*pi), -cos(q2 - 0.5*pi), 0, 0],
[0, 0, 0, 1]]
```
Joint 3 `T2_3`:
```
[[cos(q3), -sin(q3), 0, 1.25000000000000],
[sin(q3), cos(q3), 0, 0],
[0, 0, 1, 0],
[0, 0, 0, 1]]
```
Joint 4 `T3_4`:
```
[[cos(q4), -sin(q4), 0, -0.0540000000000000],
[0, 0, 1, 1.50000000000000],
[-sin(q4), -cos(q4), 0, 0],
[0, 0, 0, 1]]
```

Joint 5 `T4_5`
```
[[cos(q5), -sin(q5), 0, 0],
[0, 0, -1, 0],
[sin(q5), cos(q5), 0, 0],
[0, 0, 0, 1]]
```

Joint 6 `T5_6`
```
[[cos(q6), -sin(q6), 0, 0],
[0, 0, 1, 0],
[-sin(q6), -cos(q6), 0, 0],
[0, 0, 0, 1]]
```
Joint 7 (Gripper) `T6_G`
```
[[1, 0, 0, a6],
[0, 1, 0, 0],
[0, 0, 1, 0.303000000000000],
[0, 0, 0, 1]]
```
The full transformation for the arm is thus:

```
T0_EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)
```
Which results in the matrix:

```
[[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), 1.0*a6*sin(q1)*sin(q4)*cos(q5)*cos(q6) + 1.0*a6*sin(q1)*sin(q6)*cos(q4) - 1.0*a6*sin(q4)*sin(q6)*sin(q2 + q3)*cos(q1) + 1.0*a6*sin(q5)*cos(q1)*cos(q6)*cos(q2 + q3) + 1.0*a6*sin(q2 + q3)*cos(q1)*cos(q4)*cos(q5)*cos(q6) - 0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)], [((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), ((-sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*cos(q5) - sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), (-sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), -1.0*a6*sin(q1)*sin(q4)*sin(q6)*sin(q2 + q3) + 1.0*a6*sin(q1)*sin(q5)*cos(q6)*cos(q2 + q3) + 1.0*a6*sin(q1)*sin(q2 + q3)*cos(q4)*cos(q5)*cos(q6) - 1.0*a6*sin(q4)*cos(q1)*cos(q5)*cos(q6) - 1.0*a6*sin(q6)*cos(q1)*cos(q4) + 1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)], [(-sin(q5)*sin(q2 + q3) + cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), -1.0*a6*sin(q4)*sin(q6)*cos(q2 + q3) - 1.0*a6*sin(q5)*sin(q2 + q3)*cos(q6) + 1.0*a6*cos(q4)*cos(q5)*cos(q6)*cos(q2 + q3) - 0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]]
```

If we substitute zero for all thetas, we get a matrix representing the origin position of the Kuka Arm:

```
[[0, 0, 1, 2.15300000000000],
[0, -1, 0, 0],
[1, 0, 0, 1.94600000000000],
[0, 0, 0, 1]]
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Since the last three joints in the Kuka KR210 are revolute and their joint axes intersecpt at a single point, we have a spherical wrist with joint 5 and this is our common intesection point which we call the wrist center. This allows us to kinematically decouple the IK problem into Inverse Position and Inverse Orientation. 


In summary we doing the following:
1. <b>Inverse Position Kinematics</b> - Find the first 3 joint angles (counting from the base) from the pose position  
1. <b>Inverse Orientation Kinematics</b> - Find the remaining 3 wrist joint angles from the pose orientation.

##### Inverse Position Kinematics
To obtain the position of the wrist center we can use the transaformation matrix we defined above. As it was done on the course explanation I simplfiy the homogenous transform as follows:<br>
![homogenous transform][image15]

<b>l</b>, <b>m</b>, and <b>n</b> are orthonormal vectors representing the end-factor orientation along  X, Y, Z axes of the local corordinate frame.

Since n is the vector along the z_axis of the gripper_link, we can say the following:<br>
![equations][image16] <br> 

Where 
Px, Py, Pz = end-factor poitions 
<br>Wx, Wy Wz = writst positions
<br>d6 = DH Table parameter
<br>l = end-factor lenght
<br>
To calculate <b>nx</b>, <b>ny</b>, <b>nz</b>, we need to correct for the difference between the URDF and the DH reference frames for the end-factor. 
<br>
###### Correction Rotation Matrix
Since the orientation of the gripper is differnent in the URDF and DH Parameters defnitions we need to perfrom a correction so that we can compare the homogeneous ransform between the base link and the gripper link. <br>
This problem us visually shown here: <br>
![Gripper Ref Frame 1][image5]
<br>
To resolve this you need to create a correction rotation matrix that is composed of a rotation on the z-axis by 180 degrees followed by rotation on the Y axis by -90 degrees.
<br>
Once the correctional ratation matirx is defined, next calcualted the end-factor pose with respect the the <code>base link</code>. There are various considerations and conventions that need to be looked at regarding Euler angles, and how to shoose the correct conventions. I'll do a disservices to that mateial if i try to explain all he convenstions but I will attempt to provide the sailent points in side note below. 

###### Side Note: Compositions of Rotations
1. There are two types of rotation systems:
1.1 Extrinsic rotations are performed about the fixed world references frame
1.1 Intrinsic rotations are performed about the coordinate system as rotated by the previous operation. 
<br>
This is was the transformation matracies look like for extrinsic vs intrinsic 
<br>
![IntrinsicVsEntrinsicMatrices][image21]
<br>
Last key point to close the side bar is Rotation do not follow the commutative property of multiplication. So the order of matrix multiplcation matters.
<br>
One such convention is the x-y-z extrinsic rotations. Taking this convension our roation matrix will be as follows
```
Rrpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr
```
<br> where R_corr is the Correction Rotation Matrix

Since we need roll, pitch and yaw we need to use the ``` transformations.py ``` module from the TF package to get these values since ROS returns then in quaternions. <br>

We then extract <b>nx</b>, <b>ny</b>, <b>nz</b> values from this Rrpy matrix to obtain the wrist cneter position. Once we have the wrist center poisiton. <br>
![Gripper rotation eq][image7]

We can calculate theta 1, 2 & 3 the math below maths. The picture below presents a visual representation of the the problem.<br>
![joint2,3,WC][image17]
![triangle applied to my kuka scematic][image20]
<br>
Where labels 2, 3 and WC are Joint 2, Joint 3, and the Wrist Center, respectively. 

By projecting the joints onto the z-y plane corresponding to the world reference frame, you can visualize a triangle between the three. From your DH parameters, you can calculate the distance between each joint above.
<br>

Once the wrist center is calculated we can then calcualte the first joint using a simple arch tan function e.g. <br>
![archtan eq][image9]

To calculate joints 2 and 3 we use trignometry specifically the Cosine Laws as follows:

We know the lenght of A and C. So we can just use the followwing math to calculate B <br>
![3 side equation][image18]

Once we have all the sides of the triangle we can use law of cosines as follow to calculate theta 2 and 3 <br>
![law of cosines][image19]
<br>
##### Inverse Orientation Kinematics
Here we are trying to find the final three angles theta 4 (𝜃4), theta5(𝜃5) and theta6 (𝜃6): <br>
Using the DH transforms we can obtain the resultant rotation :
```
R0_6 = R0_1R1_2R2_3R3_4R4_5*R5_6
```
Since the overall RPY rotation from base_link to gripper_link must be equal to the product of individual rotations (R0_6), the following is true: <br>
```
R0_6 = R_ee
```
R_ee is the homegenous RPY rotation between base and gripper link.

We then substitute the values we calculated for 𝜃1, 𝜃2, & 𝜃3 into the respective transformation matrices and pre-multiply boths sides by inverse of R0_3. This gives us: <br>
```
R3_6 = inv(R0_3) * R_ee
```

![archtan eq][image11]

Inverting a matrix is complex and can be numerically unstable. To avoid this issue we use the principle that rotation matrices are orthogonal and the transpose is equal to it inverse.
![archtan eq][image12]

The resultant matrix on the RHS (Right Hand Side of the equation) does not have any variables after substituting the joint angle values, and hence comparing LHS (Left Hand Side of the equation) with RHS will result in equations for theta 4 (𝜃4), theta5(𝜃5) and theta6 (𝜃6).

##### Here is the code that shows how each angle is calculated
Theta 1 `theta1`
```
theta1 = atan2(WC[1], WC[0])
```
Find the 3rd sides of the triangle
```
A = 1.501        
B = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
C = 1.25
````
Cosine Laws SSS to find all inner angles of the triangle
```
a = acos((B * B + C * C - A * A) / (2 * B * C))
b = acos((A * A + C * C - B * B) / (2 * A * C))
c = acos((A * A + B * B - C * C) / (2 * A * B))
```
Find `theta2` and `theta3`
```
theta2 = pi/2 - a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
theta3 = pi/2 - (b+0.036) # 0.036 accounts for sag in link4 of -0.054m
```
Extract rotation matrices from the transformation matrices also
extract rotation matrix R0_3 from transformation matrix T0_3 then substitute angles q1-3
```
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})
```                       
Get rotation matrix R3_6 from (transpose of R0_3 * R_EE)
```
R3_6 = R0_3.transpose() * R_ee
```
Euler angles from rotation matrix
<br>
Theta 5 `theta5`
```
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
```            
Select best solution based on theta5
<br>
Theta `theta4` & `theta6`
 ```
if (theta5 > pi) :
    theta4 = atan2(-R3_6[2,2], R3_6[0,2])
    theta6 = atan2(R3_6[1,1],-R3_6[1,0])
else:
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta6 = atan2(-R3_6[1,1],R3_6[1,0])
```


### Project Implementation

#### 1. Fill in the `IK_server.py` File with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

##### Implementation 
All my Kinematics code is in the following file `/kuka_arm/scripts/Kinematics.py`. In my IK_server my code does the following:
1. Initiate a version of my Kinematics object and call the handle_calculate_ik function to calculate the inverse kinematics.
2. Once I get a valid end-effector pose I do the following:
* create the symbolic links
* define the individual transformation matrices for ' T0_1, T1_2, T2_3, T3_4, T4_5, T5_6,  T6_G, T0_EE '
3. Then I enter a loop where the IK is calculated
* In this loop I calculate wrist center to do this the following is done:
** Construct R_ee from the roll pitch and yaw and the end-effector position. 
** This is passed to the method called 'kinematics.calculate_wrist_center(roll, pitch, yaw, px, py, pz) ' that performs the correction I describe above
** Using the R_x, R_y and R_z matrices and R_ee it calculates the WC
* After the WC is calculated the calculate thetas function is called  ' kinematics.calculate_thetas(WC, T0_1, T1_2, T2_3, R_ee, q1, q2, q3, q4, q5, q6, q7)'. Refer to the code for more details and the math used is described in 3 above.
** The code calculates theta 1, 2 & 3 first
** After that the theta 4, 5, 6 is calculated.
** Theta 4, 6 was calculated based on what the value of theta 5 to find the best angles for 4 and 6.   

##### Result Discussion 
What an amazing thing to see it pick up the object and place it in the bin. In my two runs of the code, i was able to successfully pickup 8/10 objects. 

###### Here are picture where i have a run with 9 out of 10 in the bin

![9 out of 10 with one on the floor ][image14]
<br>
![9 out of 10 in bin][image13] 
##### Areas of Improvement
1. My arm arrive at the correct point everytime but the route it takes to get there is sometimes really odd and hence it can take longer than expected to get into positon
1. This also causes the arm to knock over objects 
1. I need to investigate this as I really don't understand why this is happening.
1. The code also runs really slowly but when i run the IK_debug my code runs pretty fast. So i'm not sure if this is an issue with the VM. However i can improve the perfomace using numpy but i need to be careful as i should palce care to ensure i still have good accuracy.  

