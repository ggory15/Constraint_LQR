import numpy as np
import copy
from BackPass import backpass
from ForwardPass import forwardpass

###################### Describe the LQR problem ########################
# min x_T * Qxx_T * x_T + sigma ( x_t * Qxx_t * x_t + u_t * Quu_t * u_t)
# s.t Gx_T * x_T + gx_t = 0
#     Fx *x_t + Fu* u_t = x_t+1
########################################################################

# 1) kinematic modeling
m = 2 # state variable
n = 1 # input variable
dt = 0.01 # time interval
T = 100

Fx = np.matrix(np.eye(m));
Fx[0, 1] = dt

Fu = np.matrix(np.zeros((m, n)));
Fu[1] = dt

# 2) set gain
Qxx_t = np.matrix(np.eye(m)* 10)
Qxx_T = np.matrix(np.eye(m)* 1000)
Quu_t = np.matrix(np.eye(n)* 0.01)  # these gains is the same as example in first paper.

# 3) set final constraint
x_init = np.matrix(np.ones((m, n)))
x_final_d = np.matrix(np.zeros((m, n)))
x_final_d[0] = -1.0

Gx_T = np.matrix(np.eye(m))
gx_t = -x_final_d

# 3) do backpass !
backpass_LQR = backpass(Fx, Fu)
backpass_LQR.setGains(Qxx_t, Qxx_T, Quu_t);
backpass_LQR.setConstraint(Gx_T, gx_t)
backpass_LQR.calculate(T)

# 4) do forward pass!
forwardpass_LQR = forwardpass(Fx, Fu)
forwardpass_LQR.calculate(T, x_init, backpass_LQR.K, backpass_LQR.k)

# 5) draw result
import matplotlib.pyplot as plt
fig = plt.figure()
ax1 = fig.add_subplot(3, 1, 1)
ax2 = fig.add_subplot(3, 1, 2)
ax3 = fig.add_subplot(3, 1, 3)

for i in range(0, T):
    ax1.plot([dt*(i-1)], forwardpass_LQR.X[i][0], 'ro')
ax1.set_ylabel('x[0]')
ax1.set_xlabel('time(sec)')

for i in range(0, T):
    ax2.plot([dt*(i-1)], forwardpass_LQR.X[i][1], 'ro')
ax2.set_ylabel('x[1]')
ax2.set_xlabel('time(sec)')

for i in range(0, T):
    ax3.plot([dt*(i-1)], forwardpass_LQR.U[i][0], 'ro')
ax3.set_ylabel('u')
ax3.set_xlabel('time(sec)')








plt.show()


