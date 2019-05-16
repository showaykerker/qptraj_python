#! /usr/bin/env python

import qptraj.qptraj as qpt
import qptraj.utils as ut
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, Vector3, Point, Twist, Pose, PoseStamped
from std_msgs.msg import Header
import matplotlib.pyplot as plt
from collections import deque

pos = Point()
vel = Point()
acc = Point()
jerk = Point()
yaw = Vector3()

dt = 0.05

xy = deque(maxlen=3)

# fig = plt.figure()
# ax = fig.add_subplot(1,1,1)
def main():
	rospy.init_node('qptestpy', anonymous=True)
	# pos_pub = rospy.Publisher("/qp/pos", Point, queue_size=10)
	# vel_pub = rospy.Publisher("/qp/vel", Point, queue_size=10)
	# acc_pub = rospy.Publisher("/qp/acc", Point, queue_size=10)
	# jerk_pub = rospy.Publisher("/qp/jerk", Point, queue_size=10)
	# yaw_pub = rospy.Publisher("/qp/yaw", Vector3, queue_size=10)

	cmd_pub = rospy.Publisher("/hummingbird/command/pose", PoseStamped, queue_size=10)
	pose_sub = rospy.Subscriber("/hummingbird/ground_truth/pose", Pose, cb)

	pub_rate = rospy.Rate(1/dt)

	rospy.loginfo("Trajectory Generator Start.")

	count = 0

	plan = qpt.qptrajectory()
	path = [] # list of qpSegments


	p0 = qpt.qpTrajectoryProfile()
	p1 = qpt.qpTrajectoryProfile()
	p2 = qpt.qpTrajectoryProfile()
	p3 = qpt.qpTrajectoryProfile()
	p4 = qpt.qpTrajectoryProfile()
	p5 = qpt.qpTrajectoryProfile()
	p6 = qpt.qpTrajectoryProfile()
	p7 = qpt.qpTrajectoryProfile()
	ps = [p1, p2, p3, p4, p5, p6]
	data = [] # list of qpTrajectoryProfile

	p0.x   = np.array([0.0, 0, 0, 0])
	p0.y   = np.array([0, 0, 0, 0])
	p0.z   = np.array([1.48, 0, 0, 0])
	p0.yaw = np.array([0, 0, 0, 0])	

	p1.x   = np.array([2.88386, 0, 0, 0])
	p1.y   = np.array([-3.008638, 0, 0, 0])
	p1.z   = np.array([1.475911, 0, 0, 0])
	p1.yaw = np.array([0, 0, 0, 0])

	p2.x   = np.array([5.486382, 0, 0, 0])
	p2.y   = np.array([-3.008638, 0, 0, 0])
	p2.z   = np.array([1.476, 0, 0, 0])
	p2.yaw = np.array([0, 0, 0, 0])

	p3.x   = np.array([9.802313, 0, 0, 0])
	p3.y   = np.array([4.10093, 0, 0, 0])
	p3.z   = np.array([1.447569, 0, 0, 0])
	p3.yaw = np.array([2.33857, 0, 0, 0])

	p4.x   = np.array([7.907492, 0, 0, 0])
	p4.y   = np.array([5.818326, 0, 0, 0])
	p4.z   = np.array([1.534381, 0, 0, 0])
	p4.yaw = np.array([2.33857, 0, 0, 0])

	p5.x   = np.array([-1.112663, 0, 0, 0])
	p5.y   = np.array([6.132953, 0, 0, 0])
	p5.z   = np.array([1.475779, 0, 0, 0])
	p5.yaw = np.array([np.pi-2.416556, 0, 0, 0])
	
	p6.x   = np.array([-2.473461, 0, 0, 0])
	p6.y   = np.array([4.946811, 0, 0, 0])
	p6.z   = np.array([1.475779, 0, 0, 0])
	p6.yaw = np.array([np.pi-2.424663, 0, 0, 0])

	spd_div = 0.7
	
	p2.x[1] = p1.x[1] = (p2.x[0] - p1.x[0])/spd_div
	p2.y[1] = p1.y[1] = (p2.y[0] - p1.y[0])/spd_div
	p4.x[1] = p3.x[1] = (p4.x[0] - p3.x[0])/spd_div
	p4.y[1] = p3.y[1] = (p4.y[0] - p3.y[0])/spd_div
	p6.x[1] = p5.x[1] = (p6.x[0] - p5.x[0])/spd_div
	p6.y[1] = p5.y[1] = (p6.y[0] - p5.y[0])/spd_div

	#path.append(qpt.qpSegments(p0, p1, 2.))
	path.append(qpt.qpSegments(p1, p2, 0.5))
	path.append(qpt.qpSegments(p2, p3, 3.2))
	path.append(qpt.qpSegments(p3, p4, 0.5))
	path.append(qpt.qpSegments(p4, p5, 3.))
	path.append(qpt.qpSegments(p5, p6, 0.5))
	#path.append(qpt.qpSegments(p6, p0, 3.5))
	path.append(qpt.qpSegments(p6, p1, 3.5))


	data = plan.get_profile(path, 2.5, dt)
	max_ = len(data)

	####################################################################
	# Draw Planned Trajectory
	# planned_traj = np.zeros((len(data),4))
	# for i, d in enumerate(data):
	# 	planned_traj[i] = d.copy().numpy()
	# ax.scatter(planned_traj[:, 0], planned_traj[:, 1], c='b', marker='o', s=36)

	# # Draw WayPoints
	# defined_wp = np.zeros((len(ps), 3))
	# for i, p in enumerate(ps):
	# 	defined_wp[i] = p.x[0], p.y[0], p.yaw[0]
	# ax.scatter(defined_wp[:, 0], defined_wp[:, 1], c='r', marker='*', s=200)
	####################################################################

	# plt.ion()
	# plt.show()
	# plt.pause(1)

	
	#for i in range(max_): rospy.loginfo("\tpos %d: %s" % (i, data[i]) )

	for i in range(100):
		pub_rate.sleep();

	global xy

	

	while(not rospy.is_shutdown()):
		if count >= max_: count = 0
	
		pos.x = data[count].x[0]
		pos.y = data[count].y[0]
		pos.z = data[count].z[0]

		vel.x = data[count].x[1]
		vel.y = data[count].y[1]
		vel.z = data[count].z[1]

		acc.x = data[count].x[2]
		acc.y = data[count].y[2]
		acc.z = data[count].z[2]

		jerk.x = data[count].x[3]
		jerk.y = data[count].y[3]
		jerk.z = data[count].z[3]

		yaw.x = data[count].yaw[0]
		yaw.y = data[count].yaw[1]
		yaw.z = data[count].yaw[2]

		position = Point(pos.x, pos.y, pos.z)
		orientation = Quaternion(*ut.toQuaternion(0, 0, yaw.x))
		pose = Pose(position, orientation)
		msg = PoseStamped(Header(), pose)

		#rospy.loginfo(msg)
		rospy.loginfo('Target: %d'%count)
		#print(xy[-1])
		cmd_pub.publish(msg)
		# acc_pub.publish(acc)
		# vel_pub.publish(vel)
		# pos_pub.publish(pos)
		# jerk_pub.publish(jerk)
		# yaw_pub.publish(yaw)

		count += 1
		####################################################################
		# Draw Actual Trajectory
		# if count % 3 == 0:
		# 	ax.scatter(xy[-1][0], xy[-1][1], c='y', marker='.', s=56)
		# 	plt.show()
		# 	plt.pause(0.00001)
		####################################################################
		pub_rate.sleep()


def cb(data):
	global xy
	Yaw = ut.toEularian(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
	xy.append(np.array([data.position.x, data.position.y]))
	


if __name__ == '__main__':
	try:
		main()
	except Exception as e: print(e)
	finally:
		plt.ioff()
		plt.show()
