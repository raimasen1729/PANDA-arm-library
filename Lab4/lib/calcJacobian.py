import numpy as np
from calculateFK import FK     #change this back to "from lib.calculateFK import FK" before submitting

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    J = np.zeros((6, 7))

    ## STUDENT CODE GOES HERE
    
    fk = FK()

    joint_poses, final_matrix = fk.lab3_data(q_in)
    o_n = final_matrix[:-1, 3]
    J_v = np.zeros((3,7))
    J_w = np.zeros((3,7))

    for idx, joint_pose in enumerate(joint_poses):
        J_v[:,idx] = np.cross(joint_pose[:-1, 2], (o_n - joint_pose[:-1, 3]))
        J_w[:, idx] =  joint_pose[:-1, 2]

    J = np.concatenate((J_v, J_w), axis=0)
    return J

if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))