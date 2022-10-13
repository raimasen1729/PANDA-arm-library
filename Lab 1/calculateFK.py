import numpy as np
from math import pi
import matplotlib.pyplot as plt

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout
        
        pass

    
    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here
        jointPositions = np.zeros((8,3))
        T0e = np.identity(4)
        
        #using homogenous transformations for arbitrary angles (q)
        
        T01 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0, 0],
        [np.sin(q[0]), np.cos(q[0]), 0, 0],
        [0, 0, 1, 0.141],
        [0,0,0,1]])

        T12 = np.array([[np.cos(q[1]), -np.sin(q[1]), 0, 0],
        [ 0, 0, 1, 0],
        [-np.sin(q[1]), -np.cos(q[1]), 0, 0.192],
        [0,0,0,1]])

        T23 = np.array([[np.cos(q[2]), -np.sin(q[2]), 0, 0],
        [ 0, 0, -1, -0.195],
        [np.sin(q[2]), np.cos(q[2]), 0, 0],
        [0,0,0,1]])

        T34 = np.array([[np.sin(q[3]), np.cos(q[3]), 0, 0.082],
        [ 0, 0, -1, 0],
        [-np.cos(q[3]), np.sin(q[3]), 0, 0.121],
        [0,0,0,1]])

        T45 = np.array([[ 0, 0, -1, -0.125],
        [np.cos(q[4]), -np.sin(q[4]), 0, -0.0825],
        [-np.sin(q[4]), -np.cos(q[4]), 0, 0],
        [0,0,0,1]])

        T56 = np.array([[-np.sin(q[5]), -np.cos(q[5]), 0, 0],
        [ 0, 0, -1, 0.015],
        [np.cos(q[5]), -np.sin(q[5]), 0, 0.259],
        [0,0,0,1]])

        T67 = np.array([[ 0, 0, -1, -0.051],
        [-np.cos(q[6]), np.sin(q[6]), 0, -0.088],
        [np.sin(q[6]), np.cos(q[6]), 0, 0.015],
        [0,0,0,1]])

        T7e = np.array([[0.707, 0.707, 0, 0],
        [ -0.707, 0.707, 0, 0],
        [0, 0, 1, 0.159],
        [0,0,0,1]])

        jointPositions[0,:] = T01[:-1, 3]

        T02 = np.matmul(T01, T12)
        jointPositions[1,:] = T02[:-1, 3]

        T03 = np.matmul(T02, T23)
        jointPositions[2,:] = T03[:-1, 3]

        T04 = np.matmul(T03, T34)
        jointPositions[3,:] = T04[:-1, 3]

        T05 = np.matmul(T04, T45)
        jointPositions[4,:] = T05[:-1, 3]

        T06 = np.matmul(T05, T56)
        jointPositions[5,:] = T06[:-1, 3]

        T07 = np.matmul(T06, T67)
        jointPositions[6,:] = T07[:-1, 3]

        T0e = np.matmul(T07, T7e)
        jointPositions[7,:] = T0e[:-1, 3]

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

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(joint_positions[:,0], joint_positions[:,1], joint_positions[:,2])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
