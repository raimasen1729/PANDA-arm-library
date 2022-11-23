import numpy as np
from math import pi
#from calculateFK import FK

class IK:
    """
    Solves the 6 DOF (joint 5 fixed) IK problem for panda robot arm
    """
    # offsets along x direction 
    a1 = 0 
    a2 = 0
    a3 = 0.0825
    a4 = 0.0825
    a5 = 0 
    a6 = 0.088
    a7 = 0

    # offsets along z direction 
    d1 = 0.333
    d2 = 0 
    d3 = 0.316 
    d4 = 0
    d5 = 0.384
    d6 = 0 
    d7 = 0.210
    
    # This variable is used to express an arbitrary joint angle 
    Q0 = 0.123


    def panda_ik(self, target):
        """
        Solves 6 DOF IK problem given physical target in x, y, z space
        Args:
            target: dictionary containing:
                'R': numpy array of the end effector pose relative to the robot base 
                't': numpy array of the end effector position relative to the robot base 

        Returns:
             q = nx7 numpy array of joints in radians (q5: joint 5 angle should be 0)
        """
        q = []
        # Student's code goes in between: 
        
        # JOINT LIMITS
        lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
        upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

        #Solving by algebraic decomposition
        
        R_7_0 = target['R']
        print(R_7_0)
        t_7_0 = target['t']
        z_unit = np.array([0,0,self.d1])
    
        R_0_7 = R_7_0.T
        t_0_7 = -(R_7_0.T) @ t_7_0
        t_2_7 = t_0_7 + (R_0_7 @ z_unit)
        #print(R_0_7)
        #print(t_0_7)
        #print(t_2_7)
        
        t1 = t_2_7[0]
        t2 = t_2_7[1]
        t3 = t_2_7[2]
        
        q7 = pi/4 + (np.arctan(-t2/ t1))
        #print(q7)
        
        M = np.sin(q7-pi/4)
        K = self.d5**2 + self.d3**2 + self.a3**2 + self.a3**2
        C = np.square(t3 + self.d7) + np.square((t2/M) - self.a6) - K
        A = 0.1155
        B = 0.2290755
        q4 = -(np.arctan(C/ np.sqrt(np.square(A)+np.square(B)-np.square(C))) - np.arctan(A/ B))
        
        R = t3 + 0.21
        P = self.a3 - self.a4*np.cos(q4) - self.d3*np.sin(q4)
        Q = self.d5 - self.a4*np.sin(q4) + self.d3*np.cos(q4) 
        q6 = np.arctan(R/ np.sqrt(np.square(P)+np.square(Q)-np.square(R))) - np.arctan(P/ Q)
        
        r33_R_2_7 = -np.cos(q4 + q6)
        q2 = np.arccos(r33_R_2_7)
        
        r23_R_2_7 = -np.sin(q4 + q6) * np.sin(q7-pi/4)
        r13_R_2_7 = np.sin(q4 + q6) * np.cos(q7-pi/4)
        q3 = np.arctan(r23_R_2_7/ r13_R_2_7)
        
        r32_R_2_7 = np.sin(q3)*np.sin(q4+q6)
        r31_R_2_7 = np.sin(q4 + q6)*np.cos(q3)
        q1 = np.arctan(r32_R_2_7/ r31_R_2_7)

        q = np.zeros((1,7))
        q[0][0] = q1
        q[0][1] = q2
        q[0][2] = q3
        q[0][3] = q4
        q[0][4] = 0
        q[0][5] = q6
        q[0][6] = q7
        
        # Student's code goes in between:

        ## DO NOT EDIT THIS PART 
        # This will convert your joints output to the autograder format
        q = self.sort_joints(q)
        ## DO NOT EDIT THIS PART
        return q

    def kin_decouple(self, target):
        """
        Performs kinematic decoupling on the panda arm to find the position of wrist center
        Args: 
            target: dictionary containing:
                'R': numpy array of the end effector pose relative to the robot base 
                't': numpy array of the end effector position relative to the robot base 

        Returns:
             wrist_pos = 3x1 numpy array of the position of the wrist center in frame 7
        """
        wrist_pos = []
        return wrist_pos 

    def ik_pos(self, wrist_pos):
        """
        Solves IK position problem on the joint 4, 6, 7 
        Args: 
            wrist_pos: 3x1 numpy array of the position of the wrist center in frame 7

        Returns:
             joints_467 = nx3 numpy array of all joint angles of joint 4, 6, 7
        """
        joints_467 = []
        return joints_467

    def ik_orient(self, R, joints_467):
        """
        Solves IK orientation problem on the joint 1, 2, 3
        Args: 
            R: numpy array of the end effector pose relative to the robot base 
            joints_467: nx3 numpy array of all joint angles of joint 4, 6, 7

        Returns:
            joints_123 = nx3 numpy array of all joint angles of joint 1, 2 ,3
        """
        joints_123 = [] 
        return joint_123
    
    def sort_joints(self, q, col=0):
        """
        Sort the joint angle matrix by ascending order 
        Args: 
            q: nx7 joint angle matrix 
        Returns: 
            q_as = nx7 joint angle matrix in ascending order 
        """
        if col != 7: 
            q_as = q[q[:, col].argsort()]
            for i in range(q_as.shape[0]-1):
                if (q_as[i, col] < q_as[i+1, col]):
                    # do nothing
                    pass
                else:
                    for j in range(i+1, q_as.shape[0]):
                        if q_as[i, col] < q_as[j, col]:
                            idx = j
                            break
                        elif j == q_as.shape[0]-1:
                            idx = q_as.shape[0]

                    q_as_part = self.sort_joints(q_as[i:idx, :], col+1)
                    q_as[i:idx, :] = q_as_part
        else: 
            q_as = q[q[:, -1].argsort()]
        return q_as

def main(): 
    
    # fk solution code
    fk = FK()

    # input joints  
    q1 = 0
    q2 = 0
    q3 = 0
    q4 = -np.pi/2
    q6 = 0
    q7 = 0
    
    q_in  = np.array([q1, q2, q3, q4, 0, q6, q7])
    [_, T_fk] = fk.forward(q_in)

    # input of IK class
    target = {'R': T_fk[0:3, 0:3], 't': T_fk[0:3, 3]}
    ik = IK()
    q = ik.panda_ik(target)
    
    # verify IK solutions 
    for i in range(q.shape[0]):
        [_, T_ik] = fk.forward(q[i, :])
        print('Matrix difference = ')
        print(T_fk - T_ik)
        print()

if __name__ == '__main__':
    main()












