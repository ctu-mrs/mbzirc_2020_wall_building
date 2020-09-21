#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
import sensor_msgs.msg
import mrs_msgs.msg
import nav_msgs.msg
import tf.transformations

class GetCurrentGPSPoseState(EventState):
	'''
	Checking if gripper is connected

	-- battery_topic 	string 	topic of realsense image info
	-> output mrs_msgs.msg.TrackerPoint

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self, odometry_topic):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(GetCurrentGPSPoseState, self).__init__(
											 output_keys = ['odometry_position'],
											 outcomes=['received','not_received']
													)

		# Store state parameter for later use.
		self.max_non_connected_time_s = 1.0
		self.last_time_update = rospy.get_time()
		self.current_pose = mrs_msgs.msg.TrackerPoint()
		Logger.loginfo('[GetCurrentGPSPoseState] subscribing topic %s'%(str(odometry_topic)))
		brick_subs_ = rospy.Subscriber(odometry_topic, nav_msgs.msg.Odometry, self.odometry_callback)

		
		# The constructor is called when building the state machine, not when actually starting the behavior.
		# Thus, we cannot save the starting time now and will do so later.
		

	def odometry_callback(self, data):
		self.last_time_update = rospy.get_time()
		#self.current_pose.x = data.pose.pose.position.x
		#self.current_pose.y = data.pose.pose.position.y
		#self.current_pose.z = data.pose.pose.position.z
		#print("uaaaa")
		#quaternion = data.pose.pose.orientation
		#print(quaternion)
		#quatx= quaternion.x
        #quaty= quaternion.y
        #quatz= quaternion.z
        #quatw= quaternion.w
          
        #orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    	#(roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation_list)
        #euler = tf.transformations.euler_from_quaternion(q)
        #self.current_pose.yaw = yaw
        rospy.loginfo_throttle(5,'[GetCurrentGPSPoseState] still getting gps pose %s'%(str(self.current_pose)))

	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		
		diff_time = rospy.get_time() - self.last_time_update
		if diff_time < 1.0:
			userdata.odometry_position = self.current_pose
			Logger.loginfo('[GetCurrentGPSPoseState] returning ')
			return "received"
		else:
			Logger.logerr('[GetCurrentGPSPoseState] not received odometry in last %s seconds'%(str(diff_time)))
			return 'not_received'
			
	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		Logger.loginfo('[GetCurrentGPSPoseState] entering state')
		
