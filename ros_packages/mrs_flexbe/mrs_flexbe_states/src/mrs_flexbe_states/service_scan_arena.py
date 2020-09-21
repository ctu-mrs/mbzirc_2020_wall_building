#!/usr/bin/env python
import rospy
import arena_scan_planner.srv
import std_srvs.srv



from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 01/30/2019
@author: Robert Penicka
'''
class ServiceScanArena(EventState):
    '''
    State for scanning arena in Wall challenge in MBZIRC 2020.
        -- service_topic 	string      Service_topic_name.

        ># scanning_altitude      int16       Time that remains to finish the challenge.

        <= successed 		            Whenever the calling for successful.
        <= failed 		            When the calling failed.

    '''

    def __init__(self, service_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceScanArena, self).__init__(input_keys=[ 'scanning_robots','scanning_altitude','scanning_speed','scanning_max_acc','scanning_turning_speed','robot_id'],
                                                 output_keys=['scanning_trajectory','scanning_trajectory_start'],
                                                 outcomes = ['successed', 'failed'])

        # Store state parameter for later use.
        self._service_topic = rospy.resolve_name(service_topic)

        
        # Create proxy service client
        self._srv = ProxyServiceCaller({self._service_topic: arena_scan_planner.srv.ArenaScanTrajectory})

        self._failed = False


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._failed:
            return 'failed'
        else:
            return 'successed'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        self._failed = False

        # Check if the ProxyServiceCaller has been registered
        if not self._srv.is_available(self._service_topic):
            Logger.logerr('[ServiceScanArena]: Topic \'{}\' not yet registered!'.format(self._service_topic))
            self._failed = True
            return

        try:
            Logger.loginfo('[ServiceScanArena]: create service_request for '+self._service_topic)
            
            service_request = arena_scan_planner.srv.ArenaScanTrajectoryRequest()
            service_request.speed = userdata.scanning_speed
            service_request.acc = userdata.scanning_max_acc
            service_request.turning_speed = userdata.scanning_turning_speed
            service_request.altitude = userdata.scanning_altitude
            service_request.initial_speed = 0
            service_request.loop = False
            service_request.arena_part_id = min(userdata.robot_id+1, userdata.scanning_robots)
            service_request.num_robots = userdata.scanning_robots

            
            service_result = self._srv.call(self._service_topic, service_request)
            sweep_path = service_result.sweep_path
            userdata.scanning_trajectory = sweep_path
            userdata.scanning_trajectory_start = sweep_path[0]
            Logger.logwarn('"[ServiceScanArena]: sweep path has %d points' % (len(sweep_path)))
            Logger.logwarn('"[ServiceScanArena]: sweep path start is at %f %f %f %f' % (sweep_path[0].position.x,sweep_path[0].position.y,sweep_path[0].position.z,sweep_path[0].heading))
            if not service_result.ok:
                self._failed = True
                Logger.logwarn('"[ServiceScanArena]: Calling \'{}\' was not successful: {}'.format(self._service_topic, str(service_result.message)))
            else:
                Logger.loginfo(service_result.message)

        except Exception as e:
            Logger.logerr('"[ServiceScanArena]: Failed to call \'{}\' service request: {}'.format(self._service_topic, str(e)))
            self._failed = True
