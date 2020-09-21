#!/usr/bin/env python
import rospy
import mavros_msgs.srv

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 08/20/2019
@author: Vojtech Spurny
'''
class MavrosArmState(EventState):
    '''
    Request to arm or disarm a robot by calling the mavros service: "mavros/cmd/arming".

        -- request 	bool 	Request that specifies if a robot should be arm or disarm.

        <= finished 		Whenever the calling for successful.
        <= failed 		When the calling failed.

    '''

    def __init__(self, request):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(MavrosArmState, self).__init__(outcomes = ['finished', 'failed'])

        # Store state parameter for later use.
        self._request = request
        self._service_topic = rospy.resolve_name("mavros/cmd/arming")

        # Create proxy service client
        self._srv = ProxyServiceCaller({self._service_topic: mavros_msgs.srv.CommandBool})

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
            Logger.logerr('ProxyServiceCaller: Topic \'{}\' not yet registered!'.format(self._service_topic))
            self._failed = True
            return

        try:
            service_request = mavros_msgs.srv.CommandBoolRequest()
            service_request.value = self._request
            service_result = self._srv.call(self._service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('Calling \'{}\' was not successful: {}'.format(self._service_topic, str(service_result.result)))
            else:
                if self._request:
                    Logger.loginfo('[MavrosArmState]: Robot armed.')
                else:
                    Logger.loginfo('[MavrosArmState]: Robot disarmed.')

        except Exception as e:
            Logger.logerr('Failed to call \'{}\' service request: {}'.format(self._service_topic, str(e)))
            self._failed = True
