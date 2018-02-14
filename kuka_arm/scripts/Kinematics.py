import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, acos
from sympy.matrices import Matrix
from math import radians
class Kinematics(object):
    """description of class"""

    def transformation_matrix(self, q, d, a, alpha):
        return Matrix([[            cos(q),              -sin(q),           0,              a],
                       [ sin(q)*cos(alpha),    cos(q)*cos(alpha), -sin(alpha), -sin(alpha) *d],
                       [ sin(q)*sin(alpha),    cos(q)*sin(alpha),  cos(alpha),   cos(alpha)*d],
                       [                 0,                    0,           0,              1]])

    def get_dh_table(self, q1, q2, q3, q4, q5, q6, q7, d1, d2, d3, d4, d5, d6, d7, a0, a1, a2, a3, a4, a5, a6, 
		         alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6):
        
        ### Kuka KR210 ###
        # DH Parameters
        

        dh_table = {alpha0:      0, a0:           0, d1:      0.75, q1:        q1,
                    alpha1: -pi/2., a1:        0.35, d2:         0, q2: -pi/2.+q2, 
                    alpha2:      0, a2:        1.25, d3:         0, q3:        q3,
                    alpha3: -pi/2., a3:      -0.054, d4:       1.5, q4:        q4,
                    alpha4:  pi/2., a5:           0, d5:         0, q5:        q5,
                    alpha5: -pi/2., a4:           0, d6:         0, q6:        q6,
                    alpha6:      0, a6:           0, d7:     0.303, q7:         0}

        return  dh_table
        
    def create_individual_tf_matrices(self, q1, q2, q3, q4, q5, q6, q7, d1, d2, d3, d4, d5, d6, d7, a0, a1, a2, a3, a4, a5, a6, 
					alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6):
        ##### Homogenous Transforms
        # base_link to link 1
        dh_table = self.get_dh_table(q1, q2, q3, q4, q5, q6, q7, d1, d2, d3, d4, d5, d6, d7, a0, a1, a2, a3, a4, a5, a6, alpha0, alpha1, 				             alpha2, alpha3, alpha4, alpha5, alpha6)
        
        T0_1 = self.transformation_matrix(q1, d1, a0, alpha0).subs(dh_table)
        T1_2 = self.transformation_matrix(q2, d2, a1, alpha1).subs(dh_table)
        T2_3 = self.transformation_matrix(q3, d3, a2, alpha2).subs(dh_table)
        T3_4 = self.transformation_matrix(q4, d4, a3, alpha3).subs(dh_table)
        T4_5 = self.transformation_matrix(q5, d5, a4, alpha4).subs(dh_table)
        T5_6 = self.transformation_matrix(q6, d6, a5, alpha5).subs(dh_table)
        T6_G = self.transformation_matrix(q7, d7, a6, alpha6).subs(dh_table)
        
        T0_EE = (T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)
	# Correction Needed to Account for Orientation Difference Between
        # Definition of Gripper Link_G in URDF versus DH Convention.
        # Matrix is pre-calculated to improve performance.
        R_corr = Matrix([[0,0,1.0,0],[0,-1.0,0,0],[1.0,0,0,0],[0,0,0,1.0]])

        # Total Homogeneous Transform Between (Base) Link_0 and (End Effector) Link_7
        # With orientation correction applied
	
	# Total Homogeneous Transform between bse_link and gripper_ink with orientation correction applied
        T0_ee = simplify(T0_EE * R_corr)
        
        return T0_1, T1_2, T2_3, T3_4, T4_5, T5_6,  T6_G, T0_EE
    
    def calculate_wrist_center(self, roll, pitch, yaw, px, py, pz):
        # Composition of Homogeneous Transforms
        """T0_2 = simplify(T0_1 * T1_2)
        T0_3 = simplify(T0_2 * T2_3)
        T0_4 = simplify(T0_3 * T3_4)
        T0_5 = simplify(T0_4 * T4_5)
        T0_6 = simplify(T0_6 * T5_6)
        T0_G = simplify(T0_G * T6_G)"""

        # Correction Needed to Account of Orientation Difference Between Definiton Of
        # Gripper_Link in URDF versus DH Convention
        r, p ,y = symbols(' r p y')

        R_x = Matrix([[ 1, 0, 0],
                     [0, cos(r), -sin(r)],
                     [0, sin(r), cos(r)]]) #Roll
        R_y = Matrix([[cos(p), 0, sin(p)],
                     [0, 1, 0],
                     [-sin(p), 0, cos(p)]]) # Pitch
        R_z = Matrix([[cos(y), -sin(y), 0],
                     [sin(y), cos(y), 0],
                     [0, 0, 1]]) # Yaw

        R_ee = R_z * R_y * R_x 
        R_error = R_z.subs(y, radians(180)) * R_y.subs(p, radians(-90))
        R_ee = R_ee * R_error
        R_ee = R_ee.subs({'r': roll, 'p': pitch, 'y': yaw})

        EE = Matrix([[px],
                     [py],
                     [pz]])

        WC = EE - (0.303) * R_ee[:,2]
	
  	       
	return R_ee, WC
        
    
    def calculate_thetas(self, WC, T0_1, T1_2, T2_3, R_ee, q1, q2, q3, q4, q5, q6, q7):
	theta1 = atan2(WC[1], WC[0])
        # find the 3rd side of the triangle
	A = 1.501        
	B = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
	C = 1.25
        #Cosine Laws SSS to find all inner angles of the triangle
        a = acos((B * B + C * C - A * A) / (2 * B * C))
        b = acos((A * A + C * C - B * B) / (2 * A * C))
        c = acos((A * A + B * B - C * C) / (2 * A * B))
        #Find theta2 and theta3
        theta2 = pi/2 - a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
        theta3 = pi/2 - (b+0.036) # 0.036 accounts for sag in link4 of -0.054m

        # Extract rotation matrices from the transformation matrices
        # Extract rotation matrix R0_3 from transformation matrix T0_3 then substitute angles q1-3
        R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})
                       
        # Get rotation matrix R3_6 from (transpose of R0_3 * R_EE)
        R3_6 = R0_3.transpose() * R_ee

        # Euler angles from rotation matrix
        theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
            
        # select best solution based on theta5
        if (theta5 > pi) :
            theta4 = atan2(-R3_6[2,2], R3_6[0,2])
            theta6 = atan2(R3_6[1,1],-R3_6[1,0])
        else:
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta6 = atan2(-R3_6[1,1],R3_6[1,0])
    	
	return theta1, theta2, theta3, theta4, theta5, theta6
