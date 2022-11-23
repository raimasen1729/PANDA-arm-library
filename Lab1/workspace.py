from calculateFK import FK
import numpy as np
#from core.interfaces import ArmController

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fk = FK()

# the dictionary below contains the data returned by calling arm.joint_limits()
limits = [
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -1.7628, 'upper': 1.7628},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -3.0718, 'upper': -0.0698},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -0.0175, 'upper': 3.7525},
    {'lower': -2.8973, 'upper': 2.8973}
 ]

# TODO: create plot(s) which visualize the reachable workspace of the Panda arm,
# accounting for the joint limits.
#
# We've included some very basic plotting commands below, but you can find
# more functionality at https://matplotlib.org/stable/index.html

fig = plt.figure(figsize = (10, 8))
ax = fig.add_subplot(111, projection='3d')

# TODO: update this with real results

_, T0e_upper = fk.forward([limits[0]['upper'], limits[1]['upper'], limits[2]['upper'], limits[3]['upper'], limits[4]['upper'], limits[5]['upper'], limits[6]['upper']])
_, T0e_lower = fk.forward([limits[0]['lower'], limits[1]['lower'], limits[2]['lower'], limits[3]['lower'], limits[4]['lower'], limits[5]['lower'], limits[6]['lower']])

lower_lim_x = T0e_lower[:-1,3][0]
upper_lim_x = T0e_upper[:-1,3][0]
lower_lim_y = T0e_lower[:-1,3][1]
upper_lim_y = T0e_upper[:-1,3][1]
lower_lim_z = T0e_lower[:-1,3][2]
upper_lim_z = T0e_upper[:-1,3][2]
#print(lower_lim_x, lower_lim_y, upper_lim_x, upper_lim_y, lower_lim_z, upper_lim_z)
X = np.linspace(lower_lim_x, np.abs(upper_lim_x), 25)
Y = np.linspace(lower_lim_y, upper_lim_y, 25)
print(X)
'''
inner_R = np.sqrt(lower_lim_x**2 + lower_lim_y**2 + lower_lim_z**2)
outer_R = np.sqrt(upper_lim_x**2 + upper_lim_y**2 + upper_lim_z**2)
R_in = np.linspace(inner_R, inner_R, 25)
print(R_in)
'''
xlist=np.linspace(-1.0,1.0,50)
print(xlist)
ylist=np.linspace(-1.0,1.0,50)
r=np.linspace(1.0,1.0,50)
#print(r)
X,Y= np.meshgrid(xlist,ylist)
Z=np.sqrt(r**2-X**2-Y**2)
ax.plot_surface(X,Y,Z,color="r")
plt.title('Work Volume')
plt.show()

'''
X, Y = np.meshgrid(X, Y)
Z=np.sqrt(R_in**2-X**2-Y**2)
ax.plot_wireframe(X,Y,Z,color="r")
plt.title('The unit sphere')
plt.show()


ax.plot_surface(X, Y, R, linewidth=0)
plt.show()

xlist=np.linspace(-1.0,1.0,50)
ylist=np.linspace(-1.0,1.0,50)
r=np.linspace(1.0,1.0,50)
X,Y= np.meshgrid(xlist,ylist)
Z=sq(r**2-X**2-Y**2)
cp=ax.plot_wireframe(X,Y,Z,color="r")
plt.title('The unit sphere')
plt.show()


X = np.arange(-5, 5, 0.25)
Y = np.arange(-5, 5, 0.25)
X, Y = np.meshgrid(X, Y)
R = np.sqrt(X**2 + Y**2)
Z = np.sin(R)

# Plot the surface.
ax.plot_surface(X, Y, Z,linewidth=0, antialiased=False)
plt.show()

for i in j1:
    for j in j2:
        temp_list = [i, j, limits[2]['upper'], limits[3]['upper'], limits[4]['upper'], limits[5]['upper'], limits[6]['upper']] #outer reachable workspace
        temp_list_2 = [i, j, limits[2]['lower'], limits[3]['lower'], limits[4]['lower'], limits[5]['lower'], limits[6]['lower']] #inner reachable workspace
        _, end_effector_matrix = fk.forward(temp_list)
        _, end_effector_matrix_2 = fk.forward(temp_list_2)
        position_mat = end_effector_matrix[:-1, 3]
        print(position_mat)
        position_mat_2 = end_effector_matrix_2[:-1, 3]
        ax.plot_surface(position_mat[0], position_mat[1], position_mat[2])
        ax.plot_surface(position_mat_2[0], position_mat_2[1], position_mat_2[2])



'''