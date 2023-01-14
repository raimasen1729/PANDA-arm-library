import numpy as np
from math import pi
import matplotlib.pyplot as plt

class FK():

    def __init__(self):

        pass
        
    def lab3_data(self, q):
        Te0 = np.identity(4)
        
        T10 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0, 0], [np.sin(q[0]), np.cos(q[0]), 0, 0],[0, 0, 1, 0.141], [0,0,0,1]])
        T21 = np.array([[np.cos(q[1]), -np.sin(q[1]), 0, 0], [ 0, 0, 1, 0], [-np.sin(q[1]), -np.cos(q[1]), 0, 0.192],[0,0,0,1]])
        T32 = np.array([[np.cos(q[2]), -np.sin(q[2]), 0, 0], [ 0, 0, -1, -0.195],[np.sin(q[2]), np.cos(q[2]), 0, 0],[0,0,0,1]])
        T43 = np.array([[np.sin(q[3]), np.cos(q[3]), 0, 0.082], [ 0, 0, -1, 0], [-np.cos(q[3]), np.sin(q[3]), 0, 0.121], [0,0,0,1]])
        T54_temp = np.array([[np.cos(q[4]), -np.sin(q[4]), 0, -0.083],[ 0, 0, 1, 0.125],[-np.sin(q[4]), -np.cos(q[4]), 0, 0],[0,0,0,1]])
        T54 = np.matmul(np.array([[0, -1, 0, 0],[ 1, 0, 0, 0],[0, 0, 1, 0],[0,0,0,1]]), T54_temp)
        T65 = np.array([[-np.sin(q[5]), -np.cos(q[5]), 0, 0],[ 0, 0, -1, 0.015], [np.cos(q[5]), -np.sin(q[5]), 0, 0.259], [0,0,0,1]])
        T76_temp = np.array([[np.cos(q[6]), -np.sin(q[6]), 0, 0.088],[ 0, 0, -1, -0.051], [np.sin(q[6]), np.cos(q[6]), 0, 0.015],[0,0,0,1]])
        T76 = np.matmul(np.array([[0, 1, 0, 0],[ -1, 0, 0, 0],[0, 0, 1, 0],[0,0,0,1]]), T76_temp)
        Te7 = np.array([[0.707, 0.707, 0, 0],[ -0.707, 0.707, 0, 0],[0, 0, 1, 0.159],[0,0,0,1]])
        
        T20 = np.matmul(T10, T21)
        T30 = np.matmul(T20, T32)
        T40 = np.matmul(T30, T43)
        T50 = np.matmul(T40, T54)
        T60 = np.matmul(T50, T65)
        T70 = np.matmul(T60, T76)
        Te0 = np.matmul(T70, Te7)
        
        joint_poses = np.array([T10, T20, T30, T40, T50, T60, T70])
        #print(joint_poses.shape)
        return joint_poses, Te0     #poses means both position and orientation of each joint

    
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
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0,0,-pi/2,-pi/4,pi/2,pi,pi/4])

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
    #fig = plt.figure()
    #ax = fig.add_subplot(projection='3d')
    #ax.scatter(joint_positions[:,0], joint_positions[:,1], joint_positions[:,2])
    #ax.set_xlabel('x')
    #ax.set_ylabel('y')
    #ax.set_zlabel('z')
