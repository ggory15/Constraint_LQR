import numpy as np
import copy

class forwardpass(object):
    def __init__(self, Fx, Fu):
        self.X = []
        self.U = []
        self.Fx = Fx
        self.Fu = Fu

    # to set gains for dynamcis (in this case, kinematics)
    def setKinematics(self, Fx, Fu):
        self.Fx = Fx;
        self.Fu = Fu;

    # by using feedback and feedforward gain, calculate u and x
    def calculate(self, T, x_init, K, k):
        self.T = T

        self.X.append(x_init)
        for i in range(0, T):
            self.U.append( K[i] * self.X[i] + k[i])
            self.X.append( self.Fx * self.X[i] + self.Fu * self.U[i])

        assert len(self.X) == T+1
        assert len(self.U) == T


