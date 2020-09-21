#!/usr/bin/env python
import rospy
import std_srvs.srv

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 09/02/2020
@author: Robert Penicka
'''
class ServiceResetMapState(EventState):
    '''
    State for calling std_srvs/Trigger type of services.
        -- service_topic 	string      Service_topic_name.
        
        <= finished 		Whenever the calling for successful.
        <= failed 		When the calling failed.

    '''

    def __init__(self, service_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceResetMapState, self).__init__(outcomes = ['finished', 'failed'])

        # Store state parameter for later use.
        self._service_topic = rospy.resolve_name(service_topic)

        # Create proxy service client
        self._srv = ProxyServiceCaller({self._service_topic: std_srvs.srv.Trigger})

        self._failed = False


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._failed:
            return 'failed'
        else:
            return 'finished'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        self._failed = False

        # Check if the ProxyServiceCaller has been registered
        if not self._srv.is_available(self._service_topic):
            Logger.logerr('[ServiceResetMapState]:  Topic \'{}\' not yet registered!'.format( self._service_topic))
            self._failed = True
            return

        try:
            service_request = std_srvs.srv.TriggerRequest()
            service_result = self._srv.call(self._service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('[ServiceResetMapState]: Calling \'{}\' was not successful: {}'.format( self._service_topic, str(service_result.message)))
            else:
                Logger.loginfo('[ServiceResetMapState]: ' + str(service_result.message))

        except Exception as e:
            Logger.logerr('[ServiceResetMapState]: Failed to call \'{}\' service request: {}'.format( self._service_topic, str(e)))
            self._failed = True
