#!/usr/bin/env python

import numpy as np
from qptraj.qptraj_struct import *
from numpy.polynomial import polynomial as poly
from cvxopt import matrix, solvers


class qptrajectory:
	def __init__(self): pass
	
	def set_waypoints(self, data:'class <waypoint>') -> None: # Seems not in use
		pass

	def get_position(self, time:'float') -> 'float': # Seems not in use
		pass

	def get_profile(self, seg:'list of class <segments>', time_interval:'float', dt:'float') -> 'list of class <trajectory_profile>':
		polyx = []; polyy = [] # double
		tprofile = [] # list of trajectory_profile
		d = np.array([0, 0, 0])

		for s in seg:
			begin = profile()
			end = profile()
			begin.V = s.b_c.pos[0], s.b_c.vel[0], s.b_c.acc[0], 0
			end.V   = s.t_c.pos[0], s.t_c.vel[0], s.t_c.acc[0], 0
			polyx = self.qpsolve(begin, end, s.time_interval)

			begin.V.fill(0)
			end.V.fill(0)
			begin.V = s.b_c.pos[1], s.b_c.vel[1], s.b_c.acc[1], 0
			end.V   = s.t_c.pos[1], s.t_c.vel[1], s.t_c.acc[1], 0
			polyy = self.qpsolve(begin, end, s.time_interval)

			for j in range(int(s.time_interval/dt)+1):
				data = trajectory_profile(d, d, d, 0.01)
				t = dt * j
				data.pos = np.array([ self.polynomial(polyx, t), self.polynomial(polyy, t), 0 ])
				data.vel = np.array([ self.polynomial(polyx, t, 1), self.polynomial(polyy, t, 1), 0 ])
				data.acc = np.array([ self.polynomial(polyx, t, 2), self.polynomial(polyy, t, 2), 0 ])
				tprofile.append(data.copy())

		return tprofile


	def qpsolve(self, begin:'class <profile>', end:'class <profile', time_interval:'float') -> 'list of float':

		t = time_interval
		b = np.array([24, 120, 360, 840])
		Q = np.zeros((8, 8))
		d = np.zeros((4, 4))
		A = np.zeros((8, 8))
		B = np.zeros((8, 1))

		d = np.array([ np.array([ b[i] * b[j] * (t**(i+j+1)) / (i+j+1) for j in range(4)]) for i in range(4)])
		Q[4:, 4:] = d.copy()
		A = np.array([
			[1, 0, 0, 0, 0, 0, 0, 0],
			[0, 1, 0, 0, 0, 0, 0, 0],
			[0, 0, 2, 0, 0, 0, 0, 0],
			[0, 0, 0, 6, 0, 0, 0, 0],
			[                    (t ** (i-0)) for i in range(8)],
			[i *                 (t ** (i-1)) for i in range(8)],
			[i * (i-1) *         (t ** (i-2)) for i in range(8)],
			[i * (i-1) * (i-2) * (t ** (i-3)) for i in range(8)]
		])
		B = np.append(begin.V, end.V).reshape((8, 1))

		G = matrix(np.zeros((1, 8)), tc='d')
		h = matrix(np.zeros((1, 1)), tc='d')
		p = matrix(np.zeros((8, 1)), tc='d')

		Q = matrix(Q, tc='d')
		A = matrix(A, tc='d')
		B = matrix(B, tc='d')

		sol = solvers.qp(Q, p, G, h, A, B)

		x = sol['x']

		return list(x)

	def polynomial(self, data:'list of float', t:'double', der_times:'int'=0) -> 'double':
		return poly.polyval(x=t, c=poly.polyder(c=data, m=der_times))




	





