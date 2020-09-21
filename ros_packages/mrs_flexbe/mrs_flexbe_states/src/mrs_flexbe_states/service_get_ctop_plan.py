#!/usr/bin/env python
import rospy
import ctop_planner.srv

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 09/25/2019
@author: Vojtech Spurny
'''
class ServiceGetCtopPlan(EventState):
    '''
    State for getting a plan for robots in Wall challenge in MBZIRC 2020.
        -- service_topic 	string      Service_topic_name.

        ># num_robots             int16       Number of robots for the planner.
        ># remaining_time_s       int16       Time that remains to finish the challenge.
        ># max_planning_time_s    int16       Time until we want to finish the challenge.
        ># times_finish_actual_s  int16[]     Time until the robots will be available.
        ># builded_brick_ids      int16[]     Array of already placed bricks.

        #> plans                            Plans how to divide robots in challenge.

        <= successed 		            Whenever the calling for successful.
        <= failed 		            When the calling failed.

    '''

    def __init__(self, service_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceGetCtopPlan, self).__init__(input_keys=['num_robots', 'remaining_time_s', 'max_planning_time_s', 'times_finish_actual_s', 'builded_brick_ids'],
                                                 output_keys=['plans'],
                                                 outcomes = ['successed', 'failed'])

        # Store state parameter for later use.
        self._service_topic = rospy.resolve_name(service_topic)

        # Create proxy service client
        self._srv = ProxyServiceCaller({self._service_topic: ctop_planner.srv.PlanCTOP})

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
            Logger.logerr('ProxyServiceCaller: Topic \'{}\' not yet registered!'.format(self._service_topic))
            self._failed = True
            return

        try:

            service_request = ctop_planner.srv.PlanCTOPRequest()
            service_request.num_robots = userdata.num_robots
            service_request.remaining_time_s = userdata.remaining_time_s
            service_request.max_planning_time_s = userdata.max_planning_time_s
            service_request.times_finish_actual_s = userdata.times_finish_actual_s
            service_request.builded_brick_ids = userdata.builded_brick_ids

            service_result = self._srv.call(self._service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('Calling \'{}\' was not successful: {}'.format(self._service_topic, str(service_result.message)))
            else:
                Logger.loginfo(service_result.message)
                userdata.plans = service_result.plans

        except Exception as e:
            Logger.logerr('Failed to call \'{}\' service request: {}'.format(self._service_topic, str(e)))
            self._failed = True
