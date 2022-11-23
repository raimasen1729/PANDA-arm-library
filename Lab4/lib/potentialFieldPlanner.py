import numpy as np
from math import pi, acos
from scipy.linalg import null_space
from copy import deepcopy
'''
from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
'''
from calcJacobian import calcJacobian
from calculateFK import FK
from detectCollision import detectCollision
from loadmap import loadmap


class PotentialFieldPlanner:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    center = lower + (upper - lower) / 2 # compute middle of range of motion of each joint
    #fk = FK()

    def __init__(self, tol=1e-4, max_steps=500, min_step_size=1e-5):
        """
        Constructs a potential field planner with solver parameters.

        PARAMETERS:
        tol - the maximum distance between two joint sets
        max_steps - number of iterations before the algorithm must terminate
        min_step_size - the minimum step size before concluding that the
        optimizer has converged
        """

        # YOU MAY NEED TO CHANGE THESE PARAMETERS

        # solver parameters
        self.fk = FK()
        self.tol = tol
        self.max_steps = max_steps
        #self.min_step_size = min_step_size
        self.zeta = np.repeat(1,7)     #attractive field strength for each of the 7 joints
        self.att_thresh = 0.5                                       #can be 7 values
        self.rho_not = 0.3                                       #can be different for different objects
        self.mu = np.repeat(15,7)      #repulsive field strength for each of the 7 joints
        self.alpha = 1e-3                                        #diff for diff joints 
        
    def attractive_force(self, zeta, target, current):
        """
        INPUTS:
        target - 3x1 numpy array representing the desired joint position in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame

        OUTPUTS:
        att_f - 3x1 numpy array representing the force vector that pulls the joint 
        from the current position to the target position 
        """

        ## STUDENT CODE STARTS HERE

        att_f = np.zeros((3, 1)) 
        if (np.linalg.norm(current-target))<self.att_thresh: 
            att_f = -zeta*(current-target)
        else:
            att_f = -(current-target)/(np.linalg.norm(current-target))
        
        att_f = att_f.reshape((3,1))
        ## END STUDENT CODE

        return att_f

    def repulsive_force(self, mu, obstacle, current, unitvec=np.zeros((3,1))):
        """
        INPUTS:
        obstacle - 1x6 numpy array representing the an obstacle box in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame
        unitvec - 3x1 numpy array representing the unit vector from the current joint position 
        to the closest point on the obstacle box 

        OUTPUTS:
        rep_f - 3x1 numpy array representing the force vector that pushes the joint 
        from the obstacle
        """

        ## STUDENT CODE STARTS HERE
        
        eps = 0.00001 #to prevent denominator to be zero
        rep_f = np.zeros((3, 1))
        dist, unit = self.dist_point2box(current.reshape((3,1)).T, obstacle)
        #print(dist)
        if dist>self.rho_not:
            rep_f = 0
        else:
            rep_f = mu*((1/dist+eps) - (1/self.rho_not))*(1/dist**2+eps)*unit     
            rep_f = rep_f.T

        ## END STUDENT CODE

        return rep_f

    def dist_point2box(self, p, box):
        """
        Helper function for the computation of repulsive forces. Computes the closest point
        on the box to a given point 
    
        INPUTS:
        p - nx3 numpy array of points [x,y,z]
        box - 1x6 numpy array of minimum and maximum points of box

        OUTPUTS:
        dist - nx1 numpy array of distance between the points and the box
                dist > 0 point outside
                dist = 0 point is on or inside box
        unit - nx3 numpy array where each row is the corresponding unit vector 
        from the point to the closest spot on the box
            norm(unit) = 1 point is outside the box
            norm(unit)= 0 point is on/inside the box

         Method from MultiRRomero
         @ https://stackoverflow.com/questions/5254838/
         calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
        """
        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # Get box info
        boxMin = np.array([box[0], box[1], box[2]])
        boxMax = np.array([box[3], box[4], box[5]])
        boxCenter = boxMin*0.5 + boxMax*0.5
        p = np.array(p)

        # Get distance info from point to box boundary
        dx = np.amax(np.vstack([boxMin[0] - p[:, 0], p[:, 0] - boxMax[0], np.zeros(p[:, 0].shape)]).T, 1)
        dy = np.amax(np.vstack([boxMin[1] - p[:, 1], p[:, 1] - boxMax[1], np.zeros(p[:, 1].shape)]).T, 1)
        dz = np.amax(np.vstack([boxMin[2] - p[:, 2], p[:, 2] - boxMax[2], np.zeros(p[:, 2].shape)]).T, 1)

        # convert to distance
        distances = np.vstack([dx, dy, dz]).T
        dist = np.linalg.norm(distances, axis=1)

        # Figure out the signs
        signs = np.sign(boxCenter-p)

        # Calculate unit vector and replace with
        unit = distances / dist[:, np.newaxis] * signs
        unit[np.isnan(unit)] = 0
        unit[np.isinf(unit)] = 0
        return dist, unit

    def compute_forces(self, target, obstacle, current):
        """
        Helper function for the computation of forces on every joints. Computes the sum 
        of forces (attactive, repulsive) on each joint. 

        INPUTS:
        target - 3x7 numpy array representing the desired joint/end effector positions 
        in the world frame
        obstacle - nx6 numpy array representing the obstacle box min and max positions   #another name for map_struct
        in the world frame
        current- 3x7 numpy array representing the current joint/end effector positions 
        in the world frame

        OUTPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each 
        joint/end effector
        """

        ## STUDENT CODE STARTS HERE
        obstacle = np.array(obstacle[0])
        att_f = np.zeros((3,1))
        rep_f = np.zeros((3,1))
        joint_forces = np.zeros((3, 7))
        for joint in range(current.shape[1]):
            att_f = self.attractive_force(self.zeta[joint], target[:, joint], current[:,joint])
            for obs in range(obstacle.shape[0]):
                rep_f += self.repulsive_force(self.mu[joint], obstacle[obs,:], current[:,joint])
            joint_forces[:,joint] = att_f.reshape((3,)) + rep_f.reshape((3,))
            
        #print(joint_forces.shape) (3,7)
        
        ## END STUDENT CODE

        return joint_forces
    
    def compute_torques(self, joint_forces, q):
        """
        Helper function for converting joint forces to joint torques. Computes the sum 
        of torques on each joint.

        INPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each 
        joint/end effector
        q - 1x7 numpy array representing the current joint angles

        OUTPUTS:
        joint_torques - 1x7 numpy array representing the torques on each joint 
        """

        ## STUDENT CODE STARTS HERE

        joint_torques = np.zeros((1, 7)) 
        Jv = calcJacobian(q)[:3, :]
        #print(Jv.shape)
        for joint in range(Jv.shape[1]):
            joint_torques[:,joint] = np.sum(Jv.T @ joint_forces[:,joint]) #note : for later : change the Jv as per ed post 397
            
        #print(joint_torques.shape)
        #print(joint_torques)
        ## END STUDENT CODE
        
        return joint_torques

    def q_distance(self, target, current):
        """
        Helper function which computes the distance between any two
        vectors.

        INPUTS:
        target - 1x7 numpy array representing some joint angles
        current - 1x7 numpy array representing some joint angles

        OUTPUTS:
        distance - the distance between the target and the current joint sets 

        """

        ## STUDENT CODE STARTS HERE

        distance = np.linalg.norm(target-current)

        ## END STUDENT CODE

        return distance
    
    def compute_gradient(self, q, target, map_struct):
        """
        Computes the joint gradient step to move the current joint positions to the
        next set of joint positions which leads to a closer configuration to the goal 
        configuration 

        INPUTS:
        q - 1x7 numpy array. the current joint configuration, a "best guess" so far for the final answer
        target - 1x7 numpy array containing the desired joint angles
        map_struct - a map struct containing the obstacle box min and max positions (nx6)

        OUTPUTS:
        dq - 1x7 numpy array. a desired joint velocity to perform this task
        """

        ## STUDENT CODE STARTS HERE
        epsilon = 0.00001
        dq = np.zeros((7,))
        current, _ = self.fk.forward(q)
        current = current[:-1].T                         #(3,7)
        goal, _ = self.fk.forward(target)
        goal = goal[:-1].T                               #(3,7)
        joint_forces = self.compute_forces(goal, map_struct, current)
        joint_torques = self.compute_torques(joint_forces, q)
        for joint in range(q.shape[0]):
            dq[joint] = q[joint] + self.alpha * (joint_torques[:, joint]/(epsilon+np.linalg.norm(joint_torques[:, joint])))

        ## END STUDENT CODE
        return dq

    ###############################
    ### Potential Feild Solver  ###
    ###############################

    def plan(self, map_struct, start, goal):
        """
        Uses potential field to move the Panda robot arm from the startng configuration to
        the goal configuration.

        INPUTS:
        map_struct - a map struct containing min and max positions of obstacle boxes (nx6)
        start - 1x7 numpy array representing the starting joint angles for a configuration 
        goal - 1x7 numpy array representing the desired joint angles for a configuration

        OUTPUTS:
        q - nx7 numpy array of joint angles [q0, q1, q2, q3, q4, q5, q6]. This should contain
        all the joint angles throughout the path of the planner. The first row of q should be
        the starting joint angles and the last row of q should be the goal joint angles. 
        """
        ## STUDENT CODE STARTS HERE

        iter_count = 0
        curr_q = start
        temp = 0
        q_path = []
        q_path.append(start)   #insert start config
        beg_index = [0,1,2,3,4,5,6]
        end_index = [1,2,3,4,5,6,7]
        while self.q_distance(goal,curr_q)>=self.tol: 
            #collision check
            coll, _ = self.fk.forward(curr_q)
            beg_points = coll[beg_index, :]
            end_points = coll[end_index, :]
            #print(beg_points)
            for block in map_struct[0]:
                is_collided = detectCollision(beg_points, end_points, block)
                if any(is_collided):
                    return print("collision!")
            
            #saving old curr_q in temp
            temp = curr_q
            
            #compute gradient (new_curr)
            curr_q = self.compute_gradient(curr_q, goal, map_struct)
            
            #check if local minima
            if self.q_distance(curr_q, temp)<self.tol:
                #check if global minima,if yes then goal reached, if not then do random walk
                if self.q_distance(goal,curr_q)<self.tol:
                    print("goal reached")
                    break
                else:
                    curr_q = np.random.rand(7)
            
            q_path.append(curr_q)
            
            #increase iter value
            iter_count +=1
            if iter_count==self.max_steps:
                break

        q_path = np.stack(q_path)
        
        ## END STUDENT CODE

        return q_path

################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    np.set_printoptions(suppress=True,precision=5)

    planner = PotentialFieldPlanner()
    
    # inputs 
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    
    # potential field planning
    q_path = planner.plan(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    
    # show results
    for i in range(q_path.shape[0]):
        error = planner.q_distance(goal, q_path[i])
        print('iteration:',i,'q =', q_path[i, :], 'error={error:3.4f}'.format(error=error))

    #print("q path: ", q_path)
