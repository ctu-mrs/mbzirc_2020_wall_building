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


class SpiralAroundPointState(EventState):
    '''
    State for calling goto service in MRS system.
        -- service_topic 	string                                      Service_topic_name.

        ># goal_tracker_point       mrs_msgs.srv.ReferenceStampedSrvRequest    Target position in Reference style 
        ># frame_id                 string                                     frame id of targetreference

        <= successed 		Whenever the calling for successful.
        <= failed 		When the calling failed.

    '''

    def __init__(self, service_topic, brick_estimation_closest_wall_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(SpiralAroundPointState, self).__init__(input_keys=['start_spiral_tracker_point', 'frame_id'],
                                              outcomes=['successed', 'failed', 'over_time'])

        # Store state parameter for later use.
        self._service_topic = rospy.resolve_name(service_topic)
        self._closest_wall_topic =rospy.resolve_name(brick_estimation_closest_wall_topic)
       
        # Create proxy service client
        self._srv = ProxyServiceCaller({self._service_topic: mrs_msgs.srv.ReferenceStampedSrv})
        self._sub = rospy.Subscriber(self._closest_wall_topic, mbzirc_msgs.msg.MbzircBrick, self.nearest_wall_callback)


        self._start_point = None
        self._spiral_points = None
        self._failed = False
        self.received = False

    def nearest_wall_callback(self,message):
        #rospy.loginfo_throttle(5,"[SpiralAroundPointState]: received nearest wall %s"%(str(message.header.stamp)))
        
        #rospy.loginfo("rospy.get_rostime() %s"%(str(rospy.get_rostime())))
        #rospy.loginfo("message.header.stamp %s"%(str(message.header.stamp)))
        #rospy.loginfo("diff %s"%(str(rospy.get_rostime()-message.header.stamp)))
        if rospy.get_rostime() - message.header.stamp < rospy.Duration(secs=TIME_DURATION):
            rospy.loginfo_throttle(0.5,"[SpiralAroundPointState]: received nearest wall reacently %s"%(str(message.header.stamp)))
            self.received = True
        
    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        
        if self.received:
            rospy.loginfo("[SpiralAroundPointState]: successfully ended with detection")
            return 'successed'
        
        diff_time = rospy.get_time() - self._start_time
        if diff_time > TIME_DURATION:
            rospy.loginfo("[SpiralAroundPointState]: spiral ended with over_time without detection")
            return 'over_time'

        
        time_portion = diff_time/TIME_DURATION
        spiral_idx = int(len(self._spiral_points)*time_portion)
        spiral_idx = min(spiral_idx,len(self._spiral_points)-1)
        spiral_idx = max(spiral_idx,0)

        current_target = self._spiral_points[spiral_idx]

           # Check if the ProxyServiceCaller has been registered
        if not self._srv.is_available(self._service_topic):
            Logger.logerr('[SpiralAroundPointState]: Topic \'{}\' not yet registered!'.format(self._service_topic))
            self._failed = True
            return

        try:
            service_request = mrs_msgs.srv.ReferenceStampedSrvRequest()
            service_request.header = std_msgs.msg.Header()
            service_request.header.stamp = rospy.Time.now()
            service_request.header.seq = 0
            service_request.header.frame_id = userdata.frame_id
            service_request.reference.position.x = current_target[0]
            service_request.reference.position.y = current_target[1]
            service_request.reference.position.z = userdata.start_spiral_tracker_point.z
            service_request.reference.yaw = userdata.start_spiral_tracker_point.yaw
            service_result = self._srv.call(self._service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('[SpiralAroundPointState]: {} Calling \'{}\' was not successful: {} for reference position (x,y,z) = ({},{},{})'.format(time_portion,self._service_topic, str(service_result.message),service_request.reference.position.x,service_request.reference.position.y,service_request.reference.position.z))
            else:
                Logger.loginfo('[SpiralAroundPointState]: {}'.format(str(service_result.message)))

        except Exception as e:
            Logger.logerr('[SpiralAroundPointState]: Failed to call \'{}\' service request: {}'.format(self._service_topic, str(e)))
            self._failed = True

        if self._failed:
            return 'failed'

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        self._start_point = userdata.start_spiral_tracker_point
        start_direction = self._start_point.yaw - math.pi/2.0
        self._spiral_points = self.get_spiral_points([self._start_point.x,self._start_point.y],START_RADIUS,MAX_RADIUS,RADIUS_STEP,start_direction,ROT_STEP,NUM_ROTATIONS)
        self._start_time = rospy.get_time()

        self._failed = False
        self.received = False

     

    def get_spiral_points(self,pos,start_radius, max_radius,radius_step ,start_direction, rot_step,num_rotations):
        rot = start_direction
        radius = start_radius
        #speed = len_step/duration_s
            
        num_points = (2.0*math.pi*num_rotations)/rot_step
        spiral_points = []

        if PLOT:
            plt.plot([pos[0]],[pos[0]],'og')

        for i in range(int(num_points)):
            
            radius += radius_step
            radius = min(radius,max_radius)
            radius = max(radius,start_radius)
            rot += rot_step

            posx = pos[0] + radius * math.cos(rot)
            posy = pos[1] + radius * math.sin(rot)
            spiral_points.append([posx,posy])

            if PLOT:
                plt.plot([posx],[posy],'.r')

        if PLOT:
            plt.axis('equal')
            plt.show()

        return spiral_points