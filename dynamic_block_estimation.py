from math import pi
from threading import Thread
import time

import numpy as np
import rospy

from core.utils import time_in_seconds


class DynamicBlockEstimation:
    def __init__(self, arm, detector, fk, ik, team):
        self.arm = arm
        self.detector = detector
        self.fk = fk
        self.ik = ik
        self.team = team

        self.delay_s = 2.0  # time to wait in seconds between measurements
        if self.team == 'red':
            self.Tbw = np.array([[1.0, 0.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0, -1*self.detector.table_distance],
                                 [0.0, 0.0, 1.0, 0.0],
                                 [0.0, 0.0, 0.0, 1.0]])
            self.dynamic_midwaypoint = np.array([pi/2,0,0,-pi/2,0,pi/2,pi/4])
            self.goal_home = np.array([0.30, 0.05,  0.1, -1.5, 0.1, 1.5, 0.784])
        else:
            self.Tbw = np.array([[1.0, 0.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0, self.detector.table_distance],
                                 [0.0, 0.0, 1.0, 0.0],
                                 [0.0, 0.0, 0.0, 1.0]])
            self.dynamic_midwaypoint = np.array([-pi/2,0,0,-pi/2,0,pi/2,pi/4])
            self.goal_home = np.array([-0.30, 0.05,  0.1, -1.5, 0.1, 1.5, 0.784])


    def move_to_observation_position(self):
        # Move arm to a position that can view all the blocks on the turntable
        # this will vary based on the team
        if self.team == 'red':
            observation_pos = np.array([pi/2,
                                        -0.76012354,
                                         0.01978261,
                                        -2.34205014,
                                         0.02984053,
                                         1.54119353+pi/4,
                                         0.75344866])
                            
        else:
            observation_pos = np.array([-pi/2,
                                        -0.76012354,
                                         0.01978261,
                                        -2.34205014,
                                         0.02984053,
                                         1.54119353+pi/4,
                                         0.75344866])
        self.arm.safe_move_to_position(observation_pos)


    def _calculate_velocity(self, t1_bd, t2_bd, dt):
        angles = []

        blocks = t1_bd.keys()
        for block in blocks:
            p2 = t2_bd.get(block, None)
            if p2 is None:
                continue
            else:
                p1 = t1_bd[block]
                r1 = np.sqrt(p1[0]**2 + p1[1]**2)
                r2 = np.sqrt(p2[0]**2 + p2[1]**2)
                r = (r1 + r2) / 2
                dd = (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2
                theta = np.arccos((2*r**2 - dd) / (2*r**2))
                angles.append(theta)

        if len(angles) == 0:
            avg_angle = 0
        else:
            avg_angle = np.mean(angles)
        return avg_angle / dt


    def estimate_velocity(self):
        H_ee_camera = self.detector.get_H_ee_camera()
        _, T0e = self.fk.forward(self.arm.get_positions())
        T0c = T0e @ H_ee_camera

        # Measure the pose of the blocks at time t1
        t1_block_data = dict()
        t1 = time_in_seconds()
        for (name, pose) in self.detector.get_detections():
            if 'dynamic' in name:
                block_base = T0c @ pose
                block_world = self.Tbw @ block_base
                t1_block_data[name] = block_world[:3,-1]

        # Wait between measurements
        rospy.sleep(self.delay_s)

        # Measure the pose of the blocks at time t2
        t2_block_data = dict()
        t2 = time_in_seconds()
        for (name, pose) in self.detector.get_detections():
            if 'dynamic' in name:
                block_base = T0c @ pose
                block_world = self.Tbw @ block_base
                t2_block_data[name] = block_world[:3,-1]

        dt = t2 - t1
        velocity = self._calculate_velocity(t1_block_data, t2_block_data, dt)
        self.velocity = velocity
        return velocity


    def _get_current_block_poses_in_world_frame(self):
        H_ee_camera = self.detector.get_H_ee_camera()
        _, T0e = self.fk.forward(self.arm.get_positions())
        T0c = T0e @ H_ee_camera

        block_poses = dict()
        for (name, pose) in self.detector.get_detections():
            if 'dynamic' in name:
                block_base = T0c @ pose
                block_world = self.Tbw @ block_base
                block_poses[name] = block_world
        return block_poses


    def _predict_block_poses(self, block_poses, t):
        theta = self.velocity * t
        T = np.array([[np.cos(theta), -1*np.sin(theta), 0, 0],
                      [np.sin(theta), np.cos(theta), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1.0]])
        predicted_poses = dict()
        for name in block_poses:
            bp = block_poses[name]
            new_pose = T @ bp
            predicted_poses[name] = new_pose
        return predicted_poses


    def _get_closest_block(self, block_poses):
        closest = np.inf
        closest_pose = None
        closest_name = None
        for name in block_poses:
            bpb = np.linalg.inv(self.Tbw) @ block_poses[name]
            d = bpb[0,-1]**2 + bpb[1,-1]**2
            if d < closest:
                closest = d
                closest_pose = bpb
                closest_name = name
        return closest_pose

    @staticmethod
    def wait_function():
        rospy.sleep(12)

    def grab_nearest_block(self, t, dest):
        """ grab what will be the nearest block t seconds into the future"""
        self.move_to_observation_position()
        self.arm.exec_gripper_cmd(1.0,0)
        bp = self._get_current_block_poses_in_world_frame()
        bpf = self._predict_block_poses(bp, t)
        # calculate the joint positions to reach this block
        bb = self._get_closest_block(bpf)  # block pose in base frame of nearest block
        t1 = time_in_seconds()
        wait_thread = Thread(target=DynamicBlockEstimation.wait_function)
        wait_thread.start()
        q_goal, success, rollout = self.ik.inverse(bb, self.arm.get_positions())
        wait_thread.join()
        t2 = time_in_seconds()
        ik_time = t2 - t1
        #print('ik_time: {}'.format(ik_time))
        ## attempt to grab the nearest block
        t3 = time_in_seconds()
        self.arm.safe_move_to_position(q_goal)
        t4 = time_in_seconds()
        #print('move_time: {}'.format(t4-t3))
        #print('total_time: {}'.format(t4-t1))
        self.arm.exec_gripper_cmd(0.05,70)
        #if (self.arm.get_gripper_state()['position'] !=0 and self.arm.get_gripper_state()['force'] == 0:
            #block is not gripped go back to the observation position
            #break
        #print('grasping')
        self.arm.safe_move_to_position(self.dynamic_midwaypoint)
        self.arm.safe_move_to_position(dest)
        self.arm.exec_gripper_cmd(1.0,0)
        self.arm.safe_move_to_position(self.goal_home)
        # TODO: this didn't work
        #gs = self.arm.get_gripper_state()
        #print('force: {}'.format(gs['force']))
        #if not np.any(gs['force']):
        #    print('did not grasp')
        #    self.move_to_observation_position()
        #    return False
        #else:
        #    print('grasped')
        #    self.arm.safe_move_to_position(dest)
        #    self.arm.exec_gripper_cmd(1.0,0)
        #    return True
