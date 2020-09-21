#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
import sensor_msgs.msg
import mrs_msgs.msg



class IsGraspedState(EventState):
	'''
	Checking if gripper is connected

	-- battery_topic 	string 	topic of realsense image info

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self, gripper_diagnostics_topic):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(IsGraspedState, self).__init__(input_keys=['max_time_ungrasped'],
														outcomes=['is_connected', 'unconnected']
													)

		# Store state parameter for later use.
		brick_subs_ = rospy.Subscriber(gripper_diagnostics_topic, mrs_msgs.msg.GripperDiagnostics, self.gripper_diagnostics_callback)

		self.max_non_connected_time_s = 1.0
		self.last_time = rospy.get_time()
		# The constructor is called when building the state machine, not when actually starting the behavior.
		# Thus, we cannot save the starting time now and will do so later.
		

	def gripper_diagnostics_callback(self, data):
		if data.gripping_object:
			self.last_time = rospy.get_time()
			rospy.loginfo_throttle(2,'[IsGraspedState] gripper still connected')

	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		self.max_non_connected_time_s = userdata.max_time_ungrasped
		
		diff_time = rospy.get_time() - self.last_time
		
		if diff_time > self.max_non_connected_time_s:
			Logger.logerr('[IsGraspedState] gripper is unconnected ')
			return 'unconnected'
		else:
			Logger.loginfo('[IsGraspedState] gripper is connected')
			return 'is_connected'
			
	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		Logger.loginfo('[IsGraspedState] entering state')
		
