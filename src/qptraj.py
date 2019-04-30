import numpy as np
from numpy.polynomial import polynomial as poly
from cvxopt import matrix, solvers

class position_type:
	def __init__(self, x=0, y=0, z=0, Vec=None):
		assert Vec is None or (type(Vec) in [np.array, np.ndarray] and Vec.shape == (3,))
		self._x, self._y, self._z = self._pos = np.array([x, y, z]) if Vec is None else Vec
		
	@property
	def pos(self): return self._pos
	
	@property
	def x(self): return self._x
	
	@property
	def y(self): return self._y
	
	@property
	def z(self): return self._z
	
	@pos.setter
	def pos(self, new_pos): 
		assert (type(new_pos) in [np.array, np.ndarray] and new_pos.shape == (3,))
		self._x, self._y, self._z = self._pos = new_pos.copy()

	@x.setter
	def x(self, new_x): self._x = self._pos.x = new_x

	@y.setter
	def y(self, new_y): self._y = self._pos.y = new_y

	@z.setter
	def z(self, new_z): self._z = self._pos.z = new_z


class pose_type:
	def __init__(self, position=position_type()):
		assert type(position) == position_type
		self._position = position

	@property
	def position(self): return self._position

	@position.setter
	def position(self, new_position): 
		assert type(new_position) == position_type
		self._position = new_position


class waypoint:
	def __init__(self, time=0.0, pose=pose_type()):
		assert type(time) in [float, np.float32, np.float64]
		assert type(pose) == pose_type
		self._time = time
		self._pose = pose

	@property
	def time(self): return self._time

	@property
	def pose(self):return self._pose

	@time.setter
	def time(self, new_time):
		assert type(new_time) in [float, np.float32, np.float64]
		self._time = time

	@pose.setter
	def pose(self, new_pose):
		assert type(new_pose) == pose_type
		self._pose = new_pose


class trajectory_profile:
	def __init__(self, pos=np.zeros((3,)), vel=np.zeros((3,)), acc=np.zeros((3,)), time=0.0):
		assert type(pos) in [np.array, np.ndarray] and pos.shape == (3,)
		assert type(vel) in [np.array, np.ndarray] and vel.shape == (3,)
		assert type(acc) in [np.array, np.ndarray] and acc.shape == (3,)
		self._pos = pos
		self._vel = vel
		self._acc = acc
		self.time = time

	@property
	def pos(self): return self._pos
	
	@property
	def vel(self): return self._vel
	
	@property
	def acc(self): return self._acc
	
	@property
	def time(self): return self._time
	
	@pos.setter
	def pos(self, new_pos):
		assert type(new_pos) in [np.array, np.ndarray] and new_pos.shape == (3,)
		self._pos = new_pos

	@vel.setter
	def vel(self, new_vel):
		assert type(new_vel) in [np.array, np.ndarray] and new_vel.shape == (3,)
		self._vel = new_vel

	@acc.setter
	def acc(self, new_acc):
		assert type(new_acc) in [np.array, np.ndarray] and new_acc.shape == (3,)
		self._acc = new_acc

	@time.setter
	def time(self, new_time):
		self._time = new_time

	def __str__(self):
		return '%f, %f, %f, %f' % (self.pos, self.vel, self.acc, self.time)


class segments:
	def __init__(self, b_c=trajectory_profile(), t_c=trajectory_profile(), t=0.0):
		assert type(b_c) == type(t_c) == trajectory_profile
		assert type(t) in [float, np.float32, np.float64]
		self._b_c = b_c
		self._t_c = t_c
		self._time_interval = t;

	@property
	def b_c(self):
		return self._b_c
	
	@property
	def t_c(self):
		return self._t_c

	@property
	def time_interval(self):
		return self._time_interval
	

	@b_c.setter
	def b_c(self, new_val):
		assert type(new_val) == trajectory_profile
		self._b_c = new_val
	
	@t_c.setter
	def t_c(self, new_val):
		assert type(new_val) == trajectory_profile
		self._t_c = new_val


class profile:
	def __init__(self, p=0.0, v=0.0, a=0.0, j=0.0):
		self._position = p
		self._velocity = v
		self._acceleration = a
		self._jerk = j
		self._V = np.array([p, v, a, j])

	@property
	def V(self): return self._V

	@V.setter
	def V(self, val):
		val = np.array(val)
		assert val.shape == (4,)
		self._position, self._velocity, self._acceleration, self._jerk = self._V = val



class qptrajectory:
	def __init__(self): pass
	
	def set_waypoints(self, data:'class <waypoint>') -> None: # Seems not in use
		pass

	def get_position(self, time:'float') -> 'float': # Seems not in use
		pass

	def get_profile(self, seg:'list of class <segments>', time_interval:'float', dt:'float') -> 'list of class <trajectory_profile>':
		t = 0.0
		polyx = []; polyy = [] # double
		tprofile = [] # list of trajectory_profile
		d = np.array([0, 0, 0])
		data = trajectory_profile(d, d, d, 0.01)

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

			t = 0

			for j in range(int(s.time_interval/dt)+1):
				t = dt * j
				data.pos = np.array([ self.polynomial(polyx, t), self.polynomial(polyx, t), 0 ])
				data.vel = np.array([ self.polynomial(polyx, t, 1), self.polynomial(polyx, t, 1), 0 ])
				data.acc = np.array([ self.polynomial(polyx, t, 2), self.polynomial(polyx, t, 2), 0 ])
				tprofile.append(data)

		return tprofile


	def qpsolve(self, begin:'class <profile>', end:'class <profile', time_interval:'float') -> 'list of float':

		t = time_interval
		#b0, b1, b2, b3 = 24, 120, 360, 840
		b = np.array([24, 120, 360, 840], dtype=np.float32)
		D = np.zeros((8, 8), dtype=np.float32)
		d = np.zeros((4, 4), dtype=np.float32)
		A = np.zeros((8, 8), dtype=np.float32)
		B = np.zeros((8, 1), dtype=np.float32)

		d = np.array([ np.array([ b[i] * b[j] * (t**(i+j+1)) / (i+j+1) for j in range(4)]) for i in range(4)], dtype=np.float32)
		D[4:, 4:] = d.copy()
		A = np.array([
			[1, 0, 0, 0, 0, 0, 0, 0],
			[0, 1, 0, 0, 0, 0, 0, 0],
			[0, 0, 2, 0, 0, 0, 0, 0],
			[0, 0, 0, 6, 0, 0, 0, 0],
			[                    (t ** (i-0)) for i in range(8)],
			[i *                 (t ** (i-1)) for i in range(8)],
			[i * (i-1) *         (t ** (i-2)) for i in range(8)],
			[i * (i-1) * (i-2) * (t ** (i-3)) for i in range(8)]
		], dtype=np.float32)
		B = np.append(begin.V, end.V).reshape((8, 1))


		Q = np.zeros((8, 8))
		A_ = np.zeros((8, 8))
		B_ = np.zeros((8, 1))

		G = matrix(np.zeros((1, 8)), tc='d')
		h = matrix(np.zeros((1, 1)), tc='d')
		p = matrix(np.zeros((8, 1)), tc='d')

		for i in range(8):
			for j in range(i+1):
				Q[i][j] = D[j][i]

		for i in range(8):
			for j in range(8):
				A_[i][j] = A[j][i]

		for i in range(8):
			B_[i] = B[i][0]

		Q = matrix(Q, tc='d')
		A_ = matrix(A_, tc='d')
		B_ = matrix(B_, tc='d')

		sol = solvers.qp(Q, p, G, h, A_, B_)

		x = sol['x']

		return list(x)

	def polynomial(self, data:'list of float', t:'double', der_times:'int'=0) -> 'double':
		return poly.polyval(t, poly.polyder(data, der_times))


if __name__ == '__main__':
	p = position_type(np.array([1,2,3]))
	qp = qptrajectory()
	qp.get_profile(0, 0, 0)

	





