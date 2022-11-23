import numpy as np

a = np.array([2,1,3,5,4,3,1]).reshape((1,7))
b = np.array([5,3,8,5,7,1,5]).reshape((1,7))
att_f = np.zeros((3, 1)) 
att_f = -4*(a-b)
#print(att_f)
print(np.linalg.norm(a-b, axis = 0))

t = np.zeros((3,1))
for k in range(5):
    t+= np.array([2,6,4]).reshape((3,1))

#print(t)

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
       
       #q_path = np.array([]).reshape(0,7)
       iter_count = 0
       curr_q = start
       #posn_goal, _ = self.fk.forward(goal)
       #posn_goal = posn_goal[:-1].T #(3,7)
       q_path = []
       q_path.append(start.reshape((1,7)))   #insert start config
       while self.q_distance(goal,curr_q)>=self.tol:   
           #posn_curr_q, _ = self.fk.forward(curr_q)
           #posn_curr_q = posn_curr_q[:-1].T         #(3,7)
           #joint_forces = self.compute_forces(posn_goal, map_struct, posn_curr_q)
           #joint_torques = self.compute_torques(joint_forces, curr_q)
           curr_q = self.compute_gradient(curr_q, goal, map_struct)
           q_path.append(curr_q)
           if self.q_distance(goal,curr_q)<self.tol:
               print("goal reached")
               break
           iter_count +=1
           if iter_count==10000:
               break
       q_path = np.stack(q_path)

       # YOU NEED TO CHECK FOR COLLISIONS WITH OBSTACLES
       # TODO: Figure out how to use the provided function 

       # YOU MAY NEED TO DEAL WITH LOCAL MINIMA HERE
       # TODO: when detect a local minima, implement a random walk
       
       ## END STUDENT CODE

       return q_path