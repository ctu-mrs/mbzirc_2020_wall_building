#!/usr/bin/env python
import rospy
import mrs_msgs.srv
import mrs_msgs.msg
import std_msgs.msg

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller
from flexbe_core.proxy.proxy_subscriber_cached import ProxySubscriberCached

import rostopic
import math
import mbzirc_msgs.msg
import mrs_msgs.msg

'''
Created on 19/02/2019
@author: Robert Penicka
'''

PLOT=False

TIME_DURATION=10.0

NUM_ROTATIONS = 1.5
ROT_STEP = 0.2

START_RADIUS = 0
MAX_RADIUS = 1
RADIUS_STEP = (MAX_RADIUS-START_RADIUS) / (2.0 * math.pi * (NUM_ROTATIONS-1.0)/ROT_STEP)


class WaitForOtherDronesEnd(EventState):
    '''
    State for calling goto service in MRS system.
        -- service_topic 	string                                      Service_topic_name.

        ># goal_tracker_point       mrs_msgs.msg.ControlManagerDiagnostics.flying_normally    Target position in Reference style 
        ># frame_id                 string                                     frame id of targetreference

        <= successed 		Whenever the calling for successful.
        <= failed 		When the calling failed.

    '''

    def __init__(self, other_r_shared_diagnostics_topic, wait_after_start_s, wait_time_s):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(WaitForOtherDronesEnd, self).__init__(
                                              outcomes=['successed', 'failed'])

    
        self.diagnostics_subs_ = {}
        self.received_times = {}
        self.wait_time_s = wait_time_s
        self.wait_after_start_s = wait_after_start_s
        for idx in range(len(other_r_shared_diagnostics_topic)):
            diagnostics = other_r_shared_diagnostics_topic[idx]
            rospy.loginfo("[WaitForOtherDronesEnd]: subscribing diagnostics " + diagnostics)
            self.diagnostics_subs_[diagnostics] = rospy.Subscriber(diagnostics, mrs_msgs.msg.ControlManagerDiagnostics, self.diagnostics_callback, callback_args=[diagnostics])
            self.received_times[diagnostics] = rospy.get_time()  

    def diagnostics_callback(self,message,cb_args):
        #rospy.loginfo_throttle(5,"[SpiralAroundPointState]: received nearest wall %s"%(str(message.header.stamp)))
        
        diagnostics = cb_args[0]

        if message.flying_normally:
            self.received_times[diagnostics] = rospy.get_time()
            rospy.loginfo_throttle(5.0,"[WaitForOtherDronesEnd]: received diagnostics from %s"%(str(diagnostics)))
            
        


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        
        
        other_ended = True


        for diagnostics in self.received_times:
            if ( rospy.get_time() - self.received_times[diagnostics] ) < self.wait_time_s:
                other_ended = False
                rospy.loginfo_throttle(5,"[WaitForOtherDronesEnd]: still receiving %s"%(diagnostics))

        if rospy.get_time() - self._start_time > self.wait_after_start_s:
            rospy.loginfo_throttle(1,"[WaitForOtherDronesEnd]: waiting for other drones after start")
            if other_ended:
                rospy.loginfo("[WaitForOtherDronesEnd]: other drones ended")
                return 'successed'

        else:
            rospy.loginfo_throttle(5,"[WaitForOtherDronesEnd]: wait after enter state")

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        rospy.loginfo("[WaitForOtherDronesEnd]: entering state")
        self._start_time = rospy.get_time()
     
