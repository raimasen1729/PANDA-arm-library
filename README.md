# PANDA-arm-library
<!-- ### Date Created: 11/22/2022 -->

<!-- Contributors: Raima Sen -->


## High level overview of the functionality developed 
-  `calculateFK.py`: Calculates the end effector position w.r.t world frame for a given joint configuration of the manipulator [Code](Lab1/calculateFK.py) | [Detailed Explanation](Lab1/MEAM520_lab1-2.pdf)
-  `calculateIK6.py`: Calculates the Jacobian Matrix for a given joint configuration of the manipulator [Code](Lab2/calculateIK6.py) | [Detailed Explanation](Lab2/LAB2_MEAM520.pdf)
- `calcJacobian.py`: Calculates the joint velocities for achieving a particular end effector velocity in a given joint configuration [Code](Lab3/calcJacobian.py) | [Detailed Explanation](Lab3/MEAM520_lab3-7.pdf)
- `PotentialFieldPlanner.py`: Rapidly-exploring random tree planner in joint space for traversing between two joint configurations [Code](/lib/rrt.py) | [Detailed Explanation](/labs/lab4/meam520_lab4.pdf)



## Operating System

The simulator must be run on Ubuntu 20.04 with ROS noetic installed.

