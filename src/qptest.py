#! /usr/bin/env python

import qptraj.qptraj as qpt
import numpy as np
import rospy
from geometry_msgs.msg import Point, Vector3

pos = Point()
vel = Point()
acc = Point()
jerk = Point()
yaw = Vector3()

def main():
	rospy.init_node('qptestpy', anonymous=True)
	pos_pub = rospy.Publisher("/qp/pos", Point, queue_size=10)
	vel_pub = rospy.Publisher("/qp/vel", Point, queue_size=10)
	acc_pub = rospy.Publisher("/qp/acc", Point, queue_size=10)
	jerk_pub = rospy.Publisher("/qp/jerk", Point, queue_size=10)
	yaw_pub = rospy.Publisher("/qp/yaw", Vector3, queue_size=10)
	pub_rate = rospy.Rate(100)
	rospy.loginfo("Trajectory Generator Start.")

	count = 0

	plan = qpt.qptrajectory()
	path = [] # list of qpSegments
	p1 = qpt.qpTrajectoryProfile()
	p2 = qpt.qpTrajectoryProfile()
	p3 = qpt.qpTrajectoryProfile()
	p4 = qpt.qpTrajectoryProfile()
	data = [] # list of qpTrajectoryProfile

	p1.x   = np.array([1, 0, 0, 0])
	p1.y   = np.array([0, 0, 0, 0])
	p1.z   = np.array([3, 0, 0, 0])
	p1.yaw = np.array([0, 0, 0, 0])

	p2.x   = np.array([-1, 0, 0, 0])
	p2.y   = np.array([ 3.2, 0, 0, 0])
	p2.z   = np.array([5, 0, 0, 0])
	p2.yaw = np.array([5*np.pi/4, 0, 0, 0])

	p3.x   = np.array([-3, 0, 0, 0])
	p3.y   = np.array([5, 0, 0, 0])
	p3.z   = np.array([6, 0, 0, 0])
	p3.yaw = np.array([5*np.pi/2, 0, 0, 0])

	p4.x   = np.array([-1, 0, 0, 0])
	p4.y   = np.array([3., 0, 0, 0])
	p4.z   = np.array([4, 0, 0, 0])
	p4.yaw = np.array([5*np.pi/4, 0, 0, 0])

	path.append(qpt.qpSegments(p1, p2, 3.))
	path.append(qpt.qpSegments(p2, p3, 3.))
	path.append(qpt.qpSegments(p3, p4, 3.))
	path.append(qpt.qpSegments(p4, p1, 3.))

	data = plan.get_profile(path, 2.0, 0.02)
	max_ = len(data)

	#for i in range(max_): rospy.loginfo("\tpos %d: %s" % (i, data[i]) )

	for i in range(100):
		pub_rate.sleep();

	while(not rospy.is_shutdown()):
		if count >= max_: count = 0
		else:
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

		acc_pub.publish(acc)
		vel_pub.publish(vel)
		pos_pub.publish(pos)
		jerk_pub.publish(jerk)
		yaw_pub.publish(yaw)

		count += 1

		pub_rate.sleep()


if __name__ == '__main__':
	main()
