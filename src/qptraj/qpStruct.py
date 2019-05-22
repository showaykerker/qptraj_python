import numpy as np

class qpTrajectoryProfile:
	def __init__(self, x=np.zeros((4,)), y=np.zeros((4,)), z=np.zeros((4,)), yaw=np.zeros((4,)), time=0.):
		self.x, self.y, self.z, self.yaw = x, y, z, yaw
		self.time = time

	@property
	def x(self):
		return self._x

	@property
	def y(self):
		return self._y

	@property
	def z(self):
		return self._z

	@property
	def yaw(self):
		return self._yaw

	@property
	def time(self):
		return self._time


	@x.setter
	def x(self, val):
		assert type(val) == np.ndarray and val.shape == (4,)
		self._x = val

	@y.setter
	def y(self, val):
		assert type(val) == np.ndarray and val.shape == (4,)
		self._y = val

	@z.setter
	def z(self, val):
		assert type(val) == np.ndarray and val.shape == (4,)
		self._z = val

	@yaw.setter
	def yaw(self, val):
		assert type(val) == np.ndarray and val.shape == (4,)
		self._yaw = val

	@time.setter
	def time(self, val):
		assert type(val) in [int, float]
		self._time = val

	def copy(self):
		return qpTrajectoryProfile(x=self.x.copy(), y=self.y.copy(), z=self.z.copy(), yaw=self.yaw.copy(), time=self.time)

	def numpy(self, val_type='position'):
		if val_type == 'position': return np.array([self.x[0], self.y[0], self.z[0], self.yaw[0]])
		assert False

	def __str__(self):
		string = '<class qpTrajectoryProfile>\n'
		string += '    |%6s     |%6s     |%6s     |%8s   |\n' % ('x', 'y', 'z', 'yaw')
		string += '-' * 53 + '\n'
		for i in range(4):
			string += '|%2d |%10.5f |%10.5f |%10.5f |%10.5f |\n' % \
						(i, self.x[i], self.y[i], self.z[i], self.yaw[i])
		string += '-' * 53 + '\ntime: %f\n' % self.time
		return string


class qpSegments:
	def __init__(self, b_c=qpTrajectoryProfile(), t_c=qpTrajectoryProfile(), time_interval=0.1):
		self.b_c = b_c
		self.t_c = t_c
		self.time_interval = time_interval

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
	def b_c(self, val):
		assert type(val) == qpTrajectoryProfile
		self._b_c = val

	@t_c.setter
	def t_c(self, val):
		assert type(val) == qpTrajectoryProfile
		self._t_c = val

	@time_interval.setter
	def time_interval(self, val):
		assert type(val) in [int, float]
		self._time_interval = val

	def copy(self):
		return qpSegments(b_c=self.b_c, t_c=self.t_c, time_interval=self.time_interval)

	def __str__(self):
		string = '\n<class qpSegment>\n\n'
		string += 'b_c: '
		string += '\t' + str(self.b_c).replace('\n', '\n\t') + '\n'
		string += 't_c:'
		string += '\t' + str(self.b_c).replace('\n', '\n\t') + '\n'
		string += 'time_interval: %f\n' % self.time_interval
		return string


class qpProfile:
	def __init__(self, V=np.zeros((4, )), name=''):
		self.V = V
		self.name = name

	@property
	def V(self):
		return self._V

	@property
	def name(self):
		return self._name

	@V.setter
	def V(self, val):
		assert type(val) == np.ndarray and val.shape == (4,)
		self._V = val

	@name.setter
	def name(self, val):
		assert type(val) == str
		self._name = val

	def copy(self):
		return qpProfile(V=self.V, name=self.name)

	def __str__(self):
		string = '<class qpProfile>\n'
		string += 'Name: ' + self.name + '\n'
		string += 'V: ' + str(self.V) + '\n'
		return string


if __name__ == '__main__':
	q = qpTrajectoryProfile()
	qs = qpSegment()
	qp = qpProfile(name='showay')
	print(qp.copy())
