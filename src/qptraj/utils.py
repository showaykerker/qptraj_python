import numpy as np
import math

def toQuaternion(roll, pitch, yaw):
	cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
	cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
	cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)

	qx = cy * cp * sr - sy * sp * cr
	qy = sy * cp * sr + cy * sp * cr
	qz = sy * cp * cr - cy * sp * sr
	qw = cy * cp * cr + sy * sp * sr
	return np.array([qx, qy, qz, qw])

def toEularian(x, y, z, w):
	ysqr = y * y

	# roll (x-axis rotation)
	t0 = +2.0 * (w*x + y*z)
	t1 = +1.0 - 2.0*(x*x + ysqr)
	roll = math.atan2(t0, t1)

	# pitch (y-axis rotation)
	t2 = +2.0 * (w*y - z*x)
	if (t2 > 1.0): t2 = 1
	if (t2 < -1.0): t2 = -1.0
	pitch = math.asin(t2)

	# yaw (z-axis rotation)
	t3 = +2.0 * (w*z + x*y)
	t4 = +1.0 - 2.0 * (ysqr + z*z)
	yaw = math.atan2(t3, t4)

	return np.array([pitch, roll, yaw])