#!/usr/bin/env python
# This code save data from ROS topics to .txt files

import rospy
import rospkg

# messages
from geometry_msgs.msg import Twist, Wrench, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
import tf2_ros




class trajectory:
	def __init__(self):
		rospack = rospkg.RosPack()
		f_name = rospack.get_path('tp_2_pmr')+"/results/"+'voronoi.txt'
		self.file = open(f_name, "w+")
		rospy.init_node('save_log', anonymous=True)
		rospy.Subscriber("/base_pose_ground_truth", Odometry, self.odom)
		rospy.Subscriber("/cmd_vel", Twist, self.callback_velocity)
		self.odom = Odometry()
		self.cmdVel = Twist()
		self.start_t = rospy.get_time()
		print("saving!")
		#rospy.spin()
		

	def callback_velocity(self, data):
		self.cmdVel = data

	def odom(self, msg):
		self.odom = msg

	def run(self):
		rate = rospy.Rate(11)
		while not rospy.is_shutdown():
			# pos = self.tf.transform.translation
			# ori = self.tf.transform.rotation
			odom_pos = self.odom.pose.pose.position
			odom_ori = self.odom.pose.pose.orientation
			odom_vl = self.odom.twist.twist.linear.x
			odom_va = self.odom.twist.twist.angular.z
			cmd_vl = self.cmdVel.linear.x
			cmd_va = self.cmdVel.angular.z
			time = rospy.get_time() - self.start_t
			# print(time)

			self.file.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" % (odom_pos.x, odom_pos.y, odom_ori.x, odom_ori.y, odom_ori.z, odom_ori.w, odom_vl, odom_va, cmd_vl, cmd_va, time))
			rate.sleep()

if __name__ == '__main__':
	try:
		r = trajectory()
		r.run()
		r.file.close()
		#r.run()

	except rospy.ROSInterruptException:
		pass
		
