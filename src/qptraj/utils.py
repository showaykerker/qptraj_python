import numpy as np

def toQuaternion(roll, pitch, yaw):
	cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
	cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
	cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)

	qx = cy * cp * sr - sy * sp * cr
	qy = sy * cp * sr + cy * sp * cr
	qz = sy * cp * cr - cy * sp * sr
	qw = cy * cp * cr + sy * sp * sr
	return np.array([qx, qy, qz, qw])