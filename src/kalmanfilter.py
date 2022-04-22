# -*- coding: utf-8 -*-
# pylint: disable=invalid-name, too-many-arguments, too-many-branches,
# pylint: disable=too-many-locals, too-many-instance-attributes, too-many-lines

from __future__ import absolute_import, division

from copy import deepcopy
from math import log, exp, sqrt
import sys
import numpy as np
from numpy import dot, zeros, eye, isscalar, shape
import numpy.linalg as linalg


def reshape_z(z, dim_z, ndim):
    """ ensure z is a (dim_z, 1) shaped vector"""

    z = np.atleast_2d(z)
    if z.shape[1] == dim_z:
        z = z.T

    if z.shape != (dim_z, 1):
        raise ValueError('z (shape {}) must be convertible to shape ({}, 1)'.format(z.shape, dim_z))

    if ndim == 1:
        z = z[:, 0]

    if ndim == 0:
        z = z[0, 0]

    return z

class KalmanFilter(object):

    def __init__(self, dim_x, dim_z, dim_u):
        # if dim_x < 1:
        #     raise ValueError('dim_x must be 1 or greater')
        # if dim_z < 1:
        #     raise ValueError('dim_z must be 1 or greater')
        # if dim_u < 0:
        #     raise ValueError('dim_u must be 0 or greater')

        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u

        self.x = zeros((dim_x, 1))        # state
        self.P = eye(dim_x)               # uncertainty covariance
        self.Q = eye(dim_x)               # process uncertainty
        self.B = None                     # control transition matrix
        self.F = eye(dim_x)               # state transition matrix
        # self.H = zeros((dim_z, dim_x))    # measurement function
        self.H = np.array([[1., 0., 0.]])
        # self.R = eye(dim_z)               # measurement uncertainty
        self.R = eye(self.dim_z) * 4.0
        # self._alpha_sq = 1.               # fading memory control
        # self.M = np.zeros((dim_x, dim_z)) # process-measurement cross correlation
        self.z = np.array([[None]*self.dim_z]).T

        # gain and residual are computed during the innovation step. We
        # save them so that in case you want to inspect them for various
        # purposes
        self.K = np.zeros((dim_x, dim_z)) # kalman gain
        self.y = zeros((dim_z, 1))
        self.S = np.zeros((dim_z, dim_z)) # system uncertainty
        self.SI = np.zeros((dim_z, dim_z)) # inverse system uncertainty

        # identity matrix. Do not alter this.
        self._I = np.eye(dim_x)

        # these will always be a copy of x,P after predict() is called
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()

        # these will always be a copy of x,P after update() is called
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

        self.inv = np.linalg.inv

    def predict(self):

        # if B is None:
        #     B = self.B
        # if F is None:
        #     F = self.F
        # if Q is None:
        #     Q = self.Q
        # elif isscalar(Q):
        #     Q = eye(self.dim_x) * Q

        # x = Fx + Bu
        # if B is not None and u is not None:
        print("self.F = ", self.F)
        print("self x = ", self.x)
        print("self.B = ", self.B)
        print("self.u  = ", self.u)
        
        self.x = dot(self.F, self.x) + dot(self.B, self.u)

        # print("x after predict = ", self.x)
        # else:
            # self.x = dot(F, self.x)

        # P = FPF' + Q
        # self.P = self._alpha_sq * dot(dot(F, self.P), F.T) + Q
        self.P = dot(dot(self.F, self.P), self.F.T) + self.Q

        # save prior
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()

    def update(self, z):
        # print("z = ", z)
        # if z is None:
        #     print("inside the z is None")
        #     self.z = np.array([[None]*self.dim_z]).T
        #     self.x_post = self.x.copy()
        #     self.P_post = self.P.copy()
        #     self.y = zeros((self.dim_z, 1))
        #     return

        # if R is None:
        #     R = self.R
        #     print("inside R is None")
        # elif isscalar(R):
        #     print("inside R is scalar")
        #     R = eye(self.dim_z) * R

        # if H is None:
        #     print("inside H is None")
        #     z = reshape_z(z, self.dim_z, self.x.ndim)
        #     H = self.H

        # y = z - Hx
        # error (residual) between measurement and prediction
        self.y = z - dot(self.H, self.x)

        # common subexpression for speed
        PHT = dot(self.P, self.H.T)

        # S = HPH' + R
        # project system uncertainty into measurement space
        self.S = dot(self.H, PHT) + self.R
        self.SI = self.inv(self.S)
        # K = PH'inv(S)
        # map system uncertainty into kalman gain
        self.K = dot(PHT, self.SI)

        # x = x + Ky
        # predict new x with residual scaled by the kalman gain
        self.x = self.x + dot(self.K, self.y)

        # P = (I-KH)P(I-KH)' + KRK'
        # This is more numerically stable
        # and works for non-optimal K vs the equation
        # P = (I-KH)P usually seen in the literature.

        I_KH = self._I - dot(self.K, self.H)
        self.P = dot(dot(I_KH, self.P), I_KH.T) + dot(dot(self.K, self.R), self.K.T)

        # save measurement and posterior state
        self.z = deepcopy(z)
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()