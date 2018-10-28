# -*- coding: utf-8 -*-

#
# Plots for Sawyer and the nullspace
#

from __future__ import unicode_literals

import numpy as np
import matplotlib.pyplot as plt


folder = '../../data/'
prefix = 'sawyer_ns_'

# Read the files
path = folder + prefix
q = np.loadtxt(path+'q.txt')
xh     = np.loadtxt(path+"Hand.txt")
xhdes  = np.loadtxt(path+"Hand_des.txt")
xe     = np.loadtxt(path+"Elbow.txt")
xedes  = np.loadtxt(path+"Elbow_des.txt")
# stime = np.loadtxt(path+"time.txt")

# Set the plotting times
tf = 700
q = q[:tf,:];
#time = stime[:tf]
xh = xh[:tf,:]; xhdes = xhdes[:tf,:]; xe = xe[:tf,:]; xedes = xedes[:tf,:]; 
# elif (flag==2):
#     tf = 1000
#     q = q[:tf,:]; x = x[:tf,:]; xdes = xdes[:tf,:]; stime = stime[:tf]
# elif (flag==3):
#     # tf = 1000
#     # q = q[:tf,:]; x = x[:tf,:]; xdes = xdes[:tf,:]; stime = stime[:tf]
#     pass

# Plot the temporal joint configuration
plt.plot(q[:,0], q[:,1:])
plt.xlabel('tiempo [s]')
plt.ylabel('Valor articular [rad]')
plt.title('Evolución articular en el tiempo')
plt.legend((r'$q_1$', r'$q_2$', r'$q_3$', r'$q_4$', r'$q_5$', r'$q_6$', r'$q_7$'), loc='best')
plt.grid()
plt.show()

# ---------------------
# Hand
# ---------------------
# Plot position
#plt.plot(x[:,0], x[:,1:4])
plt.subplot(121)
plt.plot(xh[:,0], xh[:,1], linewidth=2)
plt.plot(xh[:,0], xh[:,2], linewidth=2)
plt.plot(xh[:,0], xh[:,3], linewidth=2)
plt.plot(xhdes[:,0], xhdes[:,1:4],'k--')
plt.xlabel('tiempo [s]')
plt.ylabel('posición [m]')
plt.title("Posición del efector final")
plt.legend(('x', 'y', 'z'), loc='best')
#plt.grid()
#plt.show()
# Plot orientation
plt.subplot(122)
plt.plot(xh[:,0], xh[:,4], linewidth=2)
plt.plot(xh[:,0], xh[:,5], linewidth=2)
plt.plot(xh[:,0], xh[:,6], linewidth=2)
plt.plot(xh[:,0], xh[:,7], linewidth=2)
plt.plot(xhdes[:,0], xhdes[:,4:], 'k--')
plt.xlabel('tiempo [s]')
plt.ylabel('cuaternión')
plt.title('Orientación del efector final')
plt.legend((r'$\varepsilon_w$', r'$\varepsilon_x$', r'$\varepsilon_y$',
            r'$\varepsilon_z$'), loc="best")
#plt.axis('equal')
#plt.grid()
plt.show()


# ---------------------
# Elbow
# ---------------------
# Plot position
#plt.plot(x[:,0], x[:,1:4])
plt.plot(xe[:,0], xe[:,1], linewidth=2)
plt.plot(xe[:,0], xe[:,2], linewidth=2)
plt.plot(xe[:,0], xe[:,3], linewidth=2)
plt.plot(xedes[:,0], xedes[:,1:4],'k--')
plt.xlabel('tiempo [s]')
plt.ylabel('posición [m]')
plt.title("Posición del codo")
plt.legend(('x', 'y', 'z'), loc='best')
#plt.axis('equal')
#plt.grid()
plt.show()

# Plot 3D
from mpl_toolkits.mplot3d import Axes3D
# if (flag==1):
ax = plt.figure().gca(projection='3d')
ax.plot(xe[:,1], xe[:,2], xe[:,3])
ax.plot(xedes[1:2,1], xedes[1:2,2], xedes[1:2,3], 'ro')
ax.set_xlabel(r'x [m]')
ax.set_ylabel('y [m]')
ax.set_zlabel('z [m]')
ax.text(xe[0,1], xe[0,2], xe[0,3],'inicial')
ax.text(xedes[0,1], xedes[0,2], xedes[0,3],'final')
ax.set_title('Trayectoria Cartesiana del codo')
plt.show()
# elif (flag==2):
#     #ax = plt.axes(projection='3d')
#     ax = plt.figure().gca(projection='3d')
#     ax.plot(x[:,1], x[:,2], x[:,3])
#     ax.plot(xdes[:,1], xdes[:,2], xdes[:,3], 'r')
#     ax.set_xlabel(r'x [m]')
#     ax.set_ylabel('y [m]')
#     ax.set_zlabel('z [m]')
#     ax.set_xlim3d([0.5, 1.4])
#     ax.set_zlim3d([0.2, 0.8])
#     # plt.gca().set_aspect('equal', adjustable='box')
#     # ax.set_aspect('equal'); plt.axis('scaled'); plt.axis('equal')
#     plt.show()

# elif (flag==3):
#     ax = plt.figure().gca(projection='3d')
#     ax.plot(xdes[:,1], xdes[:,2], xdes[:,3], 'r--')
#     ax.plot(x[:,1], x[:,2], x[:,3])
#     ax.set_xlabel(r'x [m]')
#     ax.set_ylabel('y [m]')
#     ax.set_zlabel('z [m]')
#     ax.set_xlim3d([0.7, 1.3])
#     ax.set_ylim3d([-0.1, 0.5])
#     ax.set_zlim3d([0.3, 0.9])
#     # plt.gca().set_aspect('equal', adjustable='box')
#     # ax.set_aspect('equal'); plt.axis('scaled'); plt.axis('equal')
#     plt.show()
