import qptraj as qpt
import numpy as np
import rospy
from geometry_msgs.msg import Point

pos = Point()
vel = Point()
acc = Point()

def main():
	rospy.init_node('qptestpy', anonymous=True)
	pos_pub = rospy.Publisher("/pos", Point, queue_size=10)
	vel_pub = rospy.Publisher("/vel", Point, queue_size=10)
	acc_pub = rospy.Publisher("/acc", Point, queue_size=10)
	pub_rate = rospy.Rate(100)
	rospy.loginfo("Trajectory Generator Start.")

	count = 0

	plan = qpt.qptrajectory()
	path = [] # list of segments
	p1 = qpt.trajectory_profile()
	p2 = qpt.trajectory_profile()
	p3 = qpt.trajectory_profile()
	data = [] # list of trajectory_profile

	p1.pos = np.array([1, 0, 0])
	p1.vel = np.array([0, 0, 0])
	p1.acc = np.array([0, 0, 0])

	p2.pos = np.array([0, 3, 0])
	p2.vel = np.array([-0.3, -0.2, 0])
	p2.acc = np.array([-0.1, -0.3, 0])

	p3.pos = np.array([-3, -1, 0])
	p3.vel = np.array([0.3, -0.1, 0])
	p3.acc = np.array([0.3, 0.2, 0])

	path.append(qpt.segments(p1, p2, 1.5))
	path.append(qpt.segments(p2, p3, 1.5))
	path.append(qpt.segments(p3, p1, 1.5))

	data = plan.get_profile(path, 0, 0.01)
	max_ = len(data)

	for i in range(max_):
		rospy.loginfo("===== pos %d =====" % i)
		rospy.loginfo(str(data[i].pos.transpose()))

	for i in range(100):
		pub_rate.sleep();

	while(not rospy.is_shutdown()):
		if count >= max_: count = 0
		else:
			pos.x = data[count].pos[0]
			pos.y = data[count].pos[1]
			pos.z = data[count].pos[2]

			vel.x = data[count].vel[0]
			vel.y = data[count].vel[1]
			vel.z = data[count].vel[2]

			acc.x = data[count].acc[0]
			acc.y = data[count].acc[1]
			acc.z = data[count].acc[2]

		acc_pub.publish(acc)
		vel_pub.publish(vel)
		pos_pub.publish(pos)

		count += 1
		pub_rate.sleep()


if __name__ == '__main__':
	main()