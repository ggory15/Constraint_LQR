import numpy as np
import copy

class backpass(object):
    def __init__(self, Fx, Fu):
        self.K = [];
        self.k = [];
        self.Fx = Fx
        self.Fu = Fu

    # to set gains for value function
    def setGains(self, Qxx_t, Qxx_T, Quu_t):
        self.Qxx_t = Qxx_t
        self.Qxx_T = Qxx_T
        self.Quu_t = Quu_t

    # to set gains for dynamcis (in this case, kinematics)
    def setKinematics(self, Fx, Fu):
        self.Fx = Fx;
        self.Fu = Fu;

    def setConstraint(self, Gx, gx):
        self.Gx_T = Gx
        self.gx_T = gx

    # by using constriant LQR, obtain the feedback and feedforward gain!
    def calculate(self, T):
        self.T = T
        m = self.Fx.shape[1]
        n = self.Fu.shape[1]

        Vxx = self.Qxx_T
        vx = np.matrix(np.zeros((m, 1)))
        Hx = self.Gx_T
        h = self.gx_T
	

        for i in range(T-1, -1, -1):
            mx = self.Fx.transpose() * vx
            mu = self.Fu.transpose() * vx
            Mxx = self.Qxx_t + self.Fx.transpose() * Vxx * self.Fx
            Muu = self.Quu_t + self.Fu.transpose() * Vxx * self.Fu
            Mux = self.Fu.transpose() * Vxx * self.Fx

            Nx = Hx * self.Fx
            Nu = Hx * self.Fu
            n = h  ## see equation 11b in second paper.

            from scipy.linalg import null_space

            Zw = null_space(Nu)
            if not Zw:
                Zw = 0; # for convinience.
                Py = 1;
                print "Constraint Matrix"
            else:
                Py = 0;

            # because the space of Zw and Py are orthogonal
            if Zw == 0:
                K = -Py * np.linalg.pinv( Nu * Py) * Nx
                k = -Py * np.linalg.pinv(Nu * Py) * n
            elif Py == 0:
                K = -Zw * np.linalg.inv( Zw.transpose() * Muu * Zw) * Zw.transpose() * Mux
                k = -Zw * np.linalg.inv( Zw.transpose() * Muu * Zw) * Zw.transpose() * mu

            self.K.append(K)
            self.k.append(k)

            Hx = (np.matrix(np.eye(m)) - Nu * Py * np.linalg.pinv(Nu * Py)) * Nx
            h = (np.matrix(np.eye(m)) - Nu * Py * np.linalg.pinv(Nu * Py)) * n
            Vxx = Mxx + 2*Mux.transpose() * K + K.transpose() * Muu * K
            vx = mx + K.transpose() * mu + (Mux.transpose() + K.transpose() * Muu) * k
	    Vxx = 0.5 * (Vxx + Vxx.transpose())

        self.K.reverse()
        self.k.reverse()


