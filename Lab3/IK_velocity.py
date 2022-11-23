import numpy as np 
from lib.calcJacobian import calcJacobian        #change this back to "from lib.calcJacobian import calcJacobian" before submitting



def IK_velocity(q_in, v_in, omega_in):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
         are infeasible, then dq should minimize the least squares error. If v_in
         and omega_in have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    ## STUDENT CODE GOES HERE
        
    vel_mat = np.hstack((v_in, omega_in))
    #print(vel_mat.shape)
    bool = ~np.isnan(vel_mat)
    vel_mat = vel_mat[bool]
    J = calcJacobian(q_in)
    J = J[bool]
    #print(J.shape)

    dq = np.zeros(7)

    all_solutions = np.linalg.lstsq(J, vel_mat)
    #print(all_solutions)
    dq = all_solutions[0]
    
    return dq
