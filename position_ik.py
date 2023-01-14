#IK for the final project
#using gradient descent algorithm
#currently solves only for the POSITION not ORIENTATION yet
import numpy as np
from math import pi, acos
from scipy.linalg import null_space

from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK

class IK:

    #joint limits
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    fk = FK()

    def __init__(self, linear_tol=1e-4, angular_tol=1e-3, max_steps=1000, min_step_size=1.5):

        self.linear_tol = linear_tol
        self.max_steps = max_steps
        self.alpha = min_step_size
        self.local_min_thresh = 0.01
        
    @staticmethod
    #this function takes the vector difference between the target and current POSITION
    def displacement(target, current):
        displacement = target[:-1, 3] - current[:-1, 3]
        return displacement
    
    @staticmethod
    #error function between target POSITION and current POSITION taking L2 norm as the metric
    def calc_error(target, q):
        _, curr_pose = IK.fk.forward(q)
        error = np.linalg.norm(target[:-1, 3] - curr_pose[:-1, 3], 2)
        return error
    
    #this function checks if the angles are between the joint limits and the position is reached or not
    def is_valid_solution(self, q, target):
        c1 = all(q > IK.lower)
        c2 = all(q < IK.upper)

        bool1 = (c1 and c2)

        error = IK.calc_error(target, q)
        bool2 = (error < self.linear_tol)
        success = (bool1 and bool2)
        return success

    #same method followed as in Optimization lecture
    @staticmethod
    def calc_dq(q, target):        
        _, T = IK.fk.forward(q)
        displacement = IK.displacement(target, T)
        Jac = calcJacobian(q)
        Jv = Jac[:3, :]
        dq = np.matmul(Jv.T, displacement)
        return dq


    #the inverse solver
    def inverse(self, target, seed):
        q = seed
        rollout = []
        steps = 0
        while (steps<self.max_steps and IK.calc_error(target, q) > self.linear_tol):
            rollout.append(q)
            #print("error is:", IK.calc_error(target, q))
            dq = self.calc_dq(q,target)
            q_old = q
            q = q + self.alpha * dq
            
            #implement local minima escape stuff here 
            if (abs(IK.calc_error(target, q) - IK.calc_error(target, q_old)) < self.local_min_thresh):
            	q = q + 0.3 * self.alpha * dq
            	
                        
            #if the position is obtained -> break
            if (IK.calc_error(target, q) < self.linear_tol):
            	break
            	
            steps +=1

        success = self.is_valid_solution(q, target)
        return q, success, rollout


if __name__ == "__main__":

    ik = IK()

    # matches figure in the handout
    seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    target = np.array([
        [0,-1,0,0.3],
        [-1,0,0,0],
        [0,0,-1,.5],
        [0,0,0, 1],
    ])

    q, success, rollout = ik.inverse(target, seed)

    for i, q in enumerate(rollout):
        d = IK.calc_error(target, q)
        print('iteration:',i,' q =',q, ' d={d:3.4f}'.format(d=d))

    print("Success: ",success)
    print("Solution: ",q)
    print("Iterations:", len(rollout))
