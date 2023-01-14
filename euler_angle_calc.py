from math import pi
import numpy as np

def calc_euler_angles(rotation_matrix):
    r = rotation_matrix
    if r[2][0]!=1 and r[2][0]!=-1:
    	#theta = -np.arcsin(r[2][0])
    	theta = pi - theta1
    	#psi = np.arctan2(r[2][1]/np.cos(theta), r[2][2]/np.cos(theta))
    	psi = np.arctan2(r[2][1]/np.cos(theta2), r[2][2]/np.cos(theta2))
    	#phi = np.arctan2(r[1][0]/np.cos(theta), r[0][0]/np.cos(theta))
    	phi = np.arctan2(r[1][0]/np.cos(theta2), r[0][0]/np.cos(theta2))
    else:
    	phi = 0
    	if r[2][0]==-1:
    	    theta = pi/2
    	    psi = phi + np.arctan2(r[0][1], r[0][2])
    	else:
    	    theta = -pi/2
    	    psi = -phi + np.arctan2(-r[0][1], -r[0][2]) 
       
    return psi, theta, phi	#(psi is with x axis, theta is with y axis, phi is with z axis)
    
#test case       
#r = np.array([[ 4.15336298e-01, -9.09502012e-01, -2.53631236e-05,  4.96624308e-01],
# [-9.09504036e-01, -4.15336377e-01,  2.48025105e-05, -1.13173774e-01],
# [ 6.98612060e-07,  2.08153433e-05, -9.99997975e-01,  2.25407487e-01],
# [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

#print(calc_euler_angles(r))
