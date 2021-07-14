#!/usr/bin/python2
import sys
import rospy
import baxter_interface
import image_geometry
import moveit_commander
import numpy as np
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from baxter_core_msgs.msg import EndpointState
from baxter_core_msgs.srv import(SolvePositionIK, SolvePositionIKRequest)
from std_msgs.msg import (Header, String)
from tshirt_scripts.msg import Center

#/robot/range/right_hand_range/state
#/robot/limb/right/endpoint_state

#----STEPS-----
# 1. move to tshirt view position, find coordinates of red marker (baxter_img)
# 2. convert coordinates to baxter coordinates
# 3. move baxter to x,y coordinates using Moveit planning stuff
# 4. once at desired position, twist wrist to max position
# 5. lift arm to tshirt view position (includes twisting wrist to min position again)

def getRightState(self, data):
	self.RightState = data
	print("got right state yay")

def getCenterPoint(self, data):
	self.centerPoint = data
	print("got center point yay")

class Twist(object):

	def __init__(self):
		rospy.init_node('twist_shirt')
		self.ik_service_right = rospy.ServiceProxy("ExternalTools/right/PositionKinematicsNode/IKService", SolvePositionIK)
		
		moveit_commander.roscpp_initialize('/joint_states:=/robot/joint_states')	
		self.scene = moveit_commander.PlanningSceneInterface()
		self.robot = moveit_commander.RobotCommander()
		print("hello")
		self.rgroup = moveit_commander.MoveGroupCommander("right_arm") 
		print("hello2")
		self.pose_right = Pose()
		self.pose_right.position = Point(0.83761, -0.050822, -0.027028)
		self.pose_right.orientation = Quaternion(1.0,0.0,0.0,0.0)

		rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, getRightState)
		rospy.Subscriber("/center_location", Point, getCenterPoint)

	def request_pose(self, pose, group):	
		self.pose_stamped = PoseStamped()
		self.pose_stamped.pose = pose
		self.pose_stamped.header.frame_id = "base"
		self.pose_stamped.header.stamp = rospy.Time.now()
		
		self.ik_request = SolvePositionIKRequest()
		self.ik_request.pose_stamp.append(pose_stamped)
		
		
		rospy.wait_for_service("Externaltools/right/PositionKinematicsNode/IKService")
		self.ik_response = ik_service_right(ik_request)
		if ik_response.isValid[0]:
			self.limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
			self.group1.set_start_state_to_current_state()
			self.group1.set_joint_value_target(limb_joints)
			self.theplan = group1.plan(limb_joints)
			self.group1.execute(theplan)
			return True
		else:
			return False
	
	def camera_pos(self):
		self.pose = Pose()
		self.pose.position = Point(0.83761, -0.050822, -0.027028)
		self.pose.orientation = Quaternion(1.0,0.0,0.0,0.0)
		
		request_pose(pose, rgroup)

	def twist_pos(self):
		self.pose = Pose()
		self.pose.orientation = Quaternion(1.0,0.0,0.0,0.0)
		self.pose.position = Point(bx, by, z_twist)
	
		request_pose(pose, rgroup)

	#converts 2D to 3D coords
	def pixelto3D(self):
	
		#touching table
		self.z_twist = -0.1
		#viewing position
		self.z_camera = 0.05
		#self.RightState = EndpointState()
		self.x0 = RightState.pose.position.x
		self.y0 = RightState.pose.position.y
		self.camera_z = RightState.pose.position.z
		print(camera_z) 
		
		self.pixel_size = 0.0025 #camera calibration (m/pixel) that I did not perform... pls work
		self.table_dist = 0.338 #from IR sensor (update once camera position is confirmed)
        	self.height = 400
		self.width = 640
		self.centerPoint = Center()
		self.cx = centerPoint.x
		self.cy = centerPoint.y
		#adjust offsets later
		self.x_offset = 0
		self.y_offset = 0
		#x & y baxter coordinates
		self.bx = (cx - (height/2))*pixel_size*table_dist + x0 + x_offset
		self.by = (cy - (width/2))*pixel_size*table_dist + y0 + y_offset
		print(bx)
		print(by)	
	
	def mainfunction(self):
		print("moving to vision pose...")
		self.camera_pos()
		rospy.sleep(5)
		print("moving to twisting pose..")
		self.twist_pos()

def main():
	
	TWIST = Twist()
	TWIST.mainfunction()

	rospy.spin()
		
if __name__ == '__main__':
	
	main()

	
	
