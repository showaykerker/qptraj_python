#!/usr/bin/env python

import numpy as np
import qptraj.qpStruct as qpst
from numpy.polynomial import polynomial as poly
from cvxopt import matrix, solvers

class WayPoints:
	"""
		In xyzY mode,
			pass a list of waypoints as numpy array
			with shape (n_waypointss, 4)

	"""

	def __init__(self, wps=None, time_interval=0.1, dt=0.025, type_='xyzY'):
		self.dt = dt
		if wps is not None: self.read_wps(wps, time_interval, type_)

	def read_wps(self, wps, time_interval=0.1, type_='xyzY'):
		assert type(wps) == np.ndarray and wps.shape == (wps.shape[0], 4)
		assert type(time_interval) in [np.ndarray, float]
		assert type_ == 'xyzY'

		self._wps = wps.copy()
		self._n = wps.shape[0]
		self._time_interval = np.zeros(self._n,)
		
		if type(time_interval) == float:
			self._time_interval.fill(time_interval)
		else:
			assert hasattr(time_interval, 'shape')
			assert time_interval.shape == self._time_interval.shape
			self._time_interval = time_interval.copy()

		return self._getTraj()


	def _getTraj(self) -> 'list of class <qpTrajectoryProfile>':
		assert hasattr(self, 'isSet')
		solver = qptrajectory()
		start_profile = None
		end_profile = None
		for i in range(self._n-1):

			x_start = np.array([self._wps[i][0], 0, 0, 0])
			y_start = np.array([self._wps[i][1], 0, 0, 0])
			z_start = np.array([self._wps[i][2], 0, 0, 0])
			Y_start = np.array([self._wps[i][3], 0, 0, 0])
			x_end = np.array([self._wps[i+1][0], 0, 0, 0])
			y_end = np.array([self._wps[i+1][1], 0, 0, 0])
			z_end = np.array([self._wps[i+1][2], 0, 0, 0])
			Y_end = np.array([self._wps[i+1][3], 0, 0, 0])

			start_profile = qpst.qpTrajectoryProfile(x_start, y_start, z_start, Y_start) if i == 0 else end_profile.copy()
			end_profile = qpst.qpTrajectoryProfile(x_end, y_end, z_end, Y_end)
			seg = qpst.qpSegments(b_c=start_profile, t_c=end_profile, time_interval=self._time_interval[i])

			tprofile = solver.get_profile([seg], dt=self.dt)
			trajProfile.append(tprofile)

		return trajProfile




class qptrajectory:
	def __init__(self): pass

	#def get_profile(self, seg:'list of <class qpSegment>', time_interval:'float', dt:'float') -> 'list of class <qpTrajectoryProfile>':
	def get_profile(self, seg:'list of <class qpSegment>', dt:'float') -> 'list of class <qpTrajectoryProfile>':
		tprofile = [] # list of qpTrajectoryProfile

		for s in seg:
			begin = qpProfile(s.b_c.x, name='x')
			end = qpProfile(s.t_c.x, name='x')
			polyx = self.qpsolve(begin, end, s.time_interval)

			begin = qpProfile(s.b_c.y, name='y')
			end = qpProfile(s.t_c.y, name='y')
			polyy = self.qpsolve(begin, end, s.time_interval)

			begin = qpProfile(s.b_c.z, name='z')
			end = qpProfile(s.t_c.z, name='z')
			polyz = self.qpsolve(begin, end, s.time_interval)

			begin = qpProfile(s.b_c.yaw, name='yaw')
			end = qpProfile(s.t_c.yaw, name='yaw')
			polyyaw = self.qpsolve(begin, end, s.time_interval)

			for j in range(int(s.time_interval/dt)+1):
				data = qpTrajectoryProfile(time=0.01)
				t = dt * j
				data.x = np.array([ self.polynomial(polyx, t, i_derive) for i_derive in range(4)])
				data.y = np.array([ self.polynomial(polyy, t, i_derive) for i_derive in range(4)])
				data.z = np.array([ self.polynomial(polyz, t, i_derive) for i_derive in range(4)])
				data.yaw = np.array([ self.polynomial(polyyaw, t, i_derive) for i_derive in range(4)])
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
