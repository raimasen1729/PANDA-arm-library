import numpy as np
from math import pi
import matplotlib.pyplot as plt

class FK():

    def __init__(self):

        self.a = [0, 0, 0.0825, -0.0825, 0, 0.088, 0]
        self.alpha = [-pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2, 0]
        self.d = [0.333, 0, 0.316, 0, 0.384, 0, 0.210]

    def A_calc(self, a, alpha, d, q):
        A = np.array([[np.cos(q), -np.sin(q)*np.cos(alpha), np.sin(q)*np.sin(alpha), a*np.cos(q)],
                      [np.sin(q), np.cos(q)*np.cos(alpha), -np.cos(q)*np.sin(alpha), a*np.sin(q)],
                      [0, np.sin(alpha), np.cos(alpha), d],
                      [0, 0, 0, 1]])
        
        return A
    
    def forward_vel_data(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
        OUTPUTS:
        jointPositions - 7 x 3 matrix, where each row corresponds to a rotational joint of the robot
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your code starts here
        T0_1 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0, 0],
        [np.sin(q[0]), np.cos(q[0]), 0, 0],
        [0, 0, 1, 0.141],
        [0,0,0,1]])

        T1_2 = np.array([[np.cos(q[1]), -np.sin(q[1]), 0, 0],
        [ 0, 0, 1, 0],
        [-np.sin(q[1]), -np.cos(q[1]), 0, 0.192],
        [0,0,0,1]])

        T2_3 = np.array([[np.cos(q[2]), -np.sin(q[2]), 0, 0],
        [ 0, 0, -1, -0.195],
        [np.sin(q[2]), np.cos(q[2]), 0, 0],
        [0,0,0,1]])

        T3_4 = np.array([[np.sin(q[3]), np.cos(q[3]), 0, 0.082],
        [ 0, 0, -1, 0],
        [-np.cos(q[3]), np.sin(q[3]), 0, 0.121],
        [0,0,0,1]])

        T4_5 = np.array([[np.cos(q[4]), -np.sin(q[4]), 0, -0.083],
        [ 0, 0, 1, 0.125],
        [-np.sin(q[4]), -np.cos(q[4]), 0, 0],
        [0,0,0,1]])


        Tpre = np.array([[0, -1, 0, 0],
        [ 1, 0, 0, 0],
        [0, 0, 1, 0],
        [0,0,0,1]])

        T4_5 = np.matmul(Tpre,T4_5)

        T5_6 = np.array([[-np.sin(q[5]), -np.cos(q[5]), 0, 0],
        [ 0, 0, -1, 0.015],
        [np.cos(q[5]), -np.sin(q[5]), 0, 0.259],
        [0,0,0,1]])


        T6_7 = np.array([[np.cos(q[6]), -np.sin(q[6]), 0, 0.088],
        [ 0, 0, -1, -0.051],
        [np.sin(q[6]), np.cos(q[6]), 0, 0.015],
        [0,0,0,1]])

        Tpre = np.array([[0, 1, 0, 0],
        [ -1, 0, 0, 0],
        [0, 0, 1, 0],
        [0,0,0,1]])

        T6_7 = np.matmul(Tpre,T6_7)




        T7_e = np.array([[1, 0, 0, 0],
        [ 0, 1, 0, 0],
        [0, 0, 1, 0.159],
        [0,0,0,1]])

        Tpre = np.array([[0.707, 0.707, 0, 0],
        [ -0.707, 0.707, 0, 0],
        [0, 0, 1, 0],
        [0,0,0,1]])

        T7_e = np.matmul(Tpre,T7_e)



        list_positions = []

        d_1 = T0_1
        list_positions.append(d_1)

        T0_2 = np.matmul(T0_1, T1_2)
        d_2 = T0_2
        list_positions.append(d_2)

        T0_3 = np.matmul(T0_2, T2_3)
        d_3 = T0_3
        list_positions.append(d_3)

        T0_4 = np.matmul(T0_3, T3_4)
        d_4 = T0_4
        list_positions.append(d_4)

        T0_5 = np.matmul(T0_4, T4_5)
        d_5 = T0_5
        list_positions.append(d_5)

        T0_6 = np.matmul(T0_5, T5_6)
        d_6 = T0_6
        list_positions.append(d_6)

        T0_7 = np.matmul(T0_6, T6_7)
        d_7 = T0_7
        list_positions.append(d_7)

        T0_e = np.matmul(T0_7, T7_e)
        d_e = T0_e

        jointPositions = np.asarray(list_positions)
        T0e = T0_e
        print(jointPositions.shape)

        # Your code ends here
        return jointPositions, T0e
    
    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
        OUTPUTS:
        jointPositions - 7 x 3 matrix, where each row corresponds to a rotational joint of the robot
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your code starts here
        T0_1 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0, 0],
        [np.sin(q[0]), np.cos(q[0]), 0, 0],
        [0, 0, 1, 0.141],
        [0,0,0,1]])

        T1_2 = np.array([[np.cos(q[1]), -np.sin(q[1]), 0, 0],
        [ 0, 0, 1, 0],
        [-np.sin(q[1]), -np.cos(q[1]), 0, 0.192],
        [0,0,0,1]])

        T2_3 = np.array([[np.cos(q[2]), -np.sin(q[2]), 0, 0],
        [ 0, 0, -1, -0.195],
        [np.sin(q[2]), np.cos(q[2]), 0, 0],
        [0,0,0,1]])

        T3_4 = np.array([[np.sin(q[3]), np.cos(q[3]), 0, 0.082],
        [ 0, 0, -1, 0],
        [-np.cos(q[3]), np.sin(q[3]), 0, 0.121],
        [0,0,0,1]])

        T4_5 = np.array([[np.cos(q[4]), -np.sin(q[4]), 0, -0.083],
        [ 0, 0, 1, 0.125],
        [-np.sin(q[4]), -np.cos(q[4]), 0, 0],
        [0,0,0,1]])


        Tpre = np.array([[0, -1, 0, 0],
        [ 1, 0, 0, 0],
        [0, 0, 1, 0],
        [0,0,0,1]])

        T4_5 = np.matmul(Tpre,T4_5)

        T5_6 = np.array([[-np.sin(q[5]), -np.cos(q[5]), 0, 0],
        [ 0, 0, -1, 0.015],
        [np.cos(q[5]), -np.sin(q[5]), 0, 0.259],
        [0,0,0,1]])


        T6_7 = np.array([[np.cos(q[6]), -np.sin(q[6]), 0, 0.088],
        [ 0, 0, -1, -0.051],
        [np.sin(q[6]), np.cos(q[6]), 0, 0.015],
        [0,0,0,1]])

        Tpre = np.array([[0, 1, 0, 0],
        [ -1, 0, 0, 0],
        [0, 0, 1, 0],
        [0,0,0,1]])

        T6_7 = np.matmul(Tpre,T6_7)




        T7_e = np.array([[1, 0, 0, 0],
        [ 0, 1, 0, 0],
        [0, 0, 1, 0.159],
        [0,0,0,1]])

        Tpre = np.array([[0.707, 0.707, 0, 0],
        [ -0.707, 0.707, 0, 0],
        [0, 0, 1, 0],
        [0,0,0,1]])

        T7_e = np.matmul(Tpre,T7_e)



        list_positions = []

        d_1 = T0_1[:-1, 3]
        list_positions.append(d_1)

        T0_2 = np.matmul(T0_1, T1_2)
        d_2 = T0_2[:-1, 3]
        list_positions.append(d_2)

        T0_3 = np.matmul(T0_2, T2_3)
        d_3 = T0_3[:-1, 3]
        list_positions.append(d_3)

        T0_4 = np.matmul(T0_3, T3_4)
        d_4 = T0_4[:-1, 3]
        list_positions.append(d_4)

        T0_5 = np.matmul(T0_4, T4_5)
        d_5 = T0_5[:-1, 3]
        list_positions.append(d_5)

        T0_6 = np.matmul(T0_5, T5_6)
        d_6 = T0_6[:-1, 3]
        list_positions.append(d_6)

        T0_7 = np.matmul(T0_6, T6_7)
        d_7 = T0_7[:-1, 3]
        list_positions.append(d_7)

        T0_e = np.matmul(T0_7, T7_e)
        d_e = T0_e[:-1, 3]

        jointPositions = np.asarray(list_positions)
        T0e = T0_e

        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1

    
    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0,0,-pi/2,-pi/4,pi/2,pi,pi/4])

    #joint_positions, T0e = fk.forward(q)
    
    print(fk.forward_vel_data(q))
    #print("Joint Positions:\n",joint_positions)
    #print("End Effector Pose:\n",T0e)
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(joint_positions[:,0], joint_positions[:,1], joint_positions[:,2])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
