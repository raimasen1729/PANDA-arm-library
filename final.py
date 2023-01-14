import sys
import numpy as np
from copy import deepcopy
from math import pi

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

from lib.calculateFK import FK
from lib.calcJacobian import calcJacobian
#use other IK solver
from lib.position_ik import IK
from lib.euler_angle_calc import *
from lib.dynamic_block_estimation import DynamicBlockEstimation

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds


if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()
    fk = FK()
    ik = IK()

    start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    arm.safe_move_to_position(start_position) # on your mark!

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE
    if team == 'red':
        T_world_base = np.array([[1,0,0,0],[0,1,0,-0.99],[0,0,1,0],[0,0,0,1]])
        T_world_goal = np.array([[1,0,0,0.562],[0,1,0,-0.799],[0,0,1,0.25],[0,0,0,1]])
        static_home = np.array([-0.30, 0.05,  0.1, -1.5, 0.1, 1.5, 0.784])
        goal_home = np.array([0.30, 0.05,  0.1, -1.5, 0.1, 1.5, 0.784])
    else:
        T_world_base = np.array([[1,0,0,0],[0,1,0,0.99],[0,0,1,0],[0,0,0,1]])
        T_world_goal = np.array([[1,0,0,0.562],[0,1,0,0.799],[0,0,1,0.25],[0,0,0,1]])
        static_home = np.array([0.30, 0.05,  0.1, -1.5, 0.1, 1.5, 0.784])
        goal_home = np.array([-0.30, 0.05,  0.1, -1.5, 0.1, 1.5, 0.784])

    mid_waypoint = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    T_base_world = np.linalg.inv(T_world_base)    
    T_base_goal = np.matmul(T_base_world, T_world_goal)
    q_valid, _, _ = ik.inverse(T_base_goal, mid_waypoint)

    dbe = DynamicBlockEstimation(arm, detector, fk, ik, team)
    dbe.move_to_observation_position()
    estimated_velocity = dbe.estimate_velocity() 
    #print('estimated_velocity: {}'.format(estimated_velocity))
    dbe.velocity = estimated_velocity


    # try stacking 3 dynamic blocks first
    #print("Ready for dynamic stacking")
    est_time = 15.7
    q_valid, _, _ = ik.inverse(T_base_goal, mid_waypoint)
    # TODO: change the destination based on if we successfully stack a block
    for i in range(1):
       res = dbe.grab_nearest_block(est_time, q_valid)


    arm.safe_move_to_position(static_home)
    ## get the transform from camera to panda_end_effector
    H_ee_camera = detector.get_H_ee_camera()

    _, T_base_ee = fk.forward(static_home)

    T_base_camera = np.matmul(T_base_ee, H_ee_camera)

    ##maybe include a while loop here so that as long as detections are read, the for loop below executes
    count = 0
    for (name, pose) in detector.get_detections():
        T_base_block = np.matmul(T_base_camera, pose)
        q_goal, _, _ = ik.inverse(T_base_block, static_home)
        arm.exec_gripper_cmd(0.07,0)
        arm.safe_move_to_position(q_goal)
        arm.exec_gripper_cmd(0.05,70)
        arm.safe_move_to_position(mid_waypoint)
        q_valid, _, _ = ik.inverse(T_base_goal, mid_waypoint)
        arm.safe_move_to_position(q_valid)
        arm.exec_gripper_cmd(0.09,0)
        arm.safe_move_to_position(goal_home)
        count = count +1 
        if count < 4:
        	arm.safe_move_to_position(static_home)
        	#update T_base_goal along the height
        	T_base_goal[2][3] = T_base_goal[2][3] + 0.05
        else:
        	T_base_goal[2][3] = T_base_goal[2][3] + 0.05
        	break
   
    #print("Ready for dynamic stacking")
    est_time = 16.7
    # TODO: keep going after dynamic blocks as long as they remain
    for i in range(20):
        q_valid, _, _ = ik.inverse(T_base_goal, mid_waypoint)
        dbe.grab_nearest_block(est_time, q_valid)
        T_base_goal[2][3] = T_base_goal[2][3] + 0.05
        
        
