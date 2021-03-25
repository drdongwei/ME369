#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float32,Float64MultiArray
import tf_conversions
from geometry_msgs.msg import PoseStamped,TwistStamped,Quaternion,Vector3
import time
from offboard.cfg import offboard_Config
from dynamic_reconfigure.server import Server

class stabilizer:
	def __init__(self,id=""):
		self.uav_id=id
		self.velocity = TwistStamped().twist.linear
		self.position = PoseStamped().pose.position
		self.az = 0
		self.kp = 0
		self.kv = 0

	def update_param(self, config, level):
		self.az = float(config['AZ'])
		self.kp = float(config['kp'])
		self.kv = float(config['kv'])
		return config

	# fetch the position of the quadrotor
	def poscallback(self,data):
		self.position=data.pose.position

	# fetch the velocity of the quadrotor
	def velcallback(self,data):
		self.velocity = data.twist.linear

	# control thread
	def control(self):
		rospy.init_node('offboard_control', anonymous=True)
		rospy.loginfo(" master:start offboard control..")
		self.srv = Server(offboard_Config, self.update_param)
		rate=rospy.Rate(20)
		rate.sleep()

		pubth=rospy.Publisher("/cmd/thrust", Float32,queue_size=10)
		pubor=rospy.Publisher("/cmd/orientation", Quaternion,queue_size=10)

		rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.poscallback)
		rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.velcallback)

		ref_pos=Vector3()
		ref_att=Vector3()
		#ref_pos.z = 0
		v_ref = [0,0,0]
		a_ref = [0,0,0]
		while not (rospy.is_shutdown()):
			a_x = 0
			a_y = 0
			a_z = self.az
			# Following is the physical conversion
			v_ref[0] = self.kp*(-self.position.x)
			v_ref[1] = 1*(-self.position.y)
			v_ref[2] = 1.5*(self.az - self.position.z)
			a_ref[0] = self.kv*(v_ref[0] - self.velocity.x)
			a_ref[1] = 2*(v_ref[1] - self.velocity.y)
			a_ref[2] = 1*(v_ref[2] - self.velocity.z)
			#a_ref[2] = v_ref[2]
			a_ref[0] = max(min(a_ref[0],3),-3)
			a_ref[1] = max(min(a_ref[1],3),-3)
			a_ref[2] = max(min(a_ref[2],3),-3)
			tr = 0.7 + (a_ref[2]/10.0)  # thrust
			xr = a_ref[0]/10.0  # roll angle
			yr = a_ref[1]/10.0   # pitch angle
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
