import numpy as np
from calculateFK_div import FK     #change this back to "from lib.calculateFK import FK" before submitting

def calcJacobian(q):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q: 0 x 7 configuration vector (of joint angles) [q0,q1,q2,q3,q4,q5,q6]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    J = np.zeros((6, 7))

    ## STUDENT CODE GOES HERE
    fk = FK()

    int_mat_list, final_matrix = fk.forward_vel_data(q)
    o_n = final_matrix[:-1, 3]
    J_v = np.zeros((3,7))
    J_w = np.zeros((3,7))
    #print(int_mat_list[4])

    for idx, joint_mat in enumerate(int_mat_list):
        J_v[:,idx] = np.cross(joint_mat[:-1, 2], (o_n - joint_mat[:-1, 3]))
        J_w[:, idx] =  joint_mat[:-1, 2]

    #print(J_v)
    #print(J_w)
    J = np.concatenate((J_v, J_w), axis=0)

    return J

if __name__ == '__main__':
    q= np.array([np.pi/2, 0, -np.pi/4, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))