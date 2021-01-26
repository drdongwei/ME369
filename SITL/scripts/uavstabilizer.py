#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float32,Float64MultiArray
import tf_conversions
from geometry_msgs.msg import PoseStamped,TwistStamped,Quaternion,Vector3
import time

class stabilizer:
	def __init__(self,id=""):
		self.uav_id=id

	# fetch the position of the quadrotor
	def poscallback(self,data):
		self.position=data.pose.position

	# fetch the velocity of the quadrotor
	def velcallback(self,data):
		self.velocity=data.twist.linear

	# control thread
	def control(self):
		rospy.init_node('offboard_control', anonymous=True)
		rospy.loginfo(" master:start offboard control..")
		rate=rospy.Rate(30)
		rate.sleep()

		pubth=rospy.Publisher("/cmd/thrust", Float32,queue_size=10)
		pubor=rospy.Publisher("/cmd/orentation", Quaternion,queue_size=10)

		rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.poscallback)
		rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.velcallback)

		ref_pos=Vector3()
		ref_att=Vector3()

		while not (rospy.is_shutdown()):
			ref_pos.z=2
			tr=0.6 # thrust
			xr=0   # roll angle
			yr=0   # pitch angle
			pubth.publish(tr)

			q=tf_conversions.transformations.quaternion_from_euler(-yr,xr,0)

			quat=Quaternion()
			quat.x=q[0]
			quat.y=q[1]
			quat.z=q[2]
			quat.w=q[3]
			pubor.publish(quat)
			rate.sleep()


if __name__ == '__main__':
	uav=stabilizer()
	uav.control()
