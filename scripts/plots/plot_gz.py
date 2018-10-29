# -*- coding: utf-8 -*-

#
# Plots for Sawyer and the nullspace
#

from __future__ import unicode_literals

import numpy as np
import matplotlib.pyplot as plt


folder = '../../data/'
prefix = 'sawyer_gz_'

# Read the files
path = folder + prefix
q = np.loadtxt(path+'q.txt')
xh     = np.loadtxt(path+"Hand.txt")
xhdes  = np.loadtxt(path+"Hand_des.txt")

# Set the plotting times
tf = 6700
q = q[:tf,:];
xh = xh[:tf,:]; xhdes = xhdes[:tf,:];

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
plt.plot(xh[:,0], xh[:,1], linewidth=2)
plt.plot(xh[:,0], xh[:,2], linewidth=2)
plt.plot(xh[:,0], xh[:,3], linewidth=2)
plt.plot(xhdes[:,0], xhdes[:,1:4],'k--')
plt.xlabel('tiempo [s]')
plt.ylabel('posición [m]')
plt.title("Posición del efector final")
plt.legend(('x', 'y', 'z'), loc='best')
plt.show()
