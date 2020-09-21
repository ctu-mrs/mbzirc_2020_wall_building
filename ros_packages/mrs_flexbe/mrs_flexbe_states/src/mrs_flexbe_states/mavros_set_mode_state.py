#!/usr/bin/env python
import rospy
import mavros_msgs.srv

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 08/20/2019
@author: Vojtech Spurny
'''
class MavrosSetModeState(EventState):
    '''
    Request to switch FCU operation mode using mavros service: "mavros/set_mode".

        -- mode      string 	Specifies to which mode the FCU shoul be switch.

        <= finished 		Whenever the switching for successful.
        <= failed 		When the switching failed.

    '''

    def __init__(self, request):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(MavrosSetModeState, self).__init__(outcomes = ['finished', 'failed'])

        # Store state parameter for later use.
        self._request = request
        self._service_topic = rospy.resolve_name("mavros/set_mode")

        # Create proxy service client
        self._srv = ProxyServiceCaller({self._service_topic: mavros_msgs.srv.SetMode})

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
            Logger.logerr('[MavrosSetMode]: ProxyServiceCaller: Topic \'{}\' not yet registered!'.format(self._service_topic))
            self._failed = True
            return

        try:
            service_request = mavros_msgs.srv.SetModeRequest()
            service_request.custom_mode = self._request
            service_result = self._srv.call(self._service_topic, service_request)

            if not service_result.mode_sent:
                self._failed = True
                Logger.logwarn('[MavrosSetMode]: Calling \'{}\' was not successful: {}'.format(self._service_topic, str(service_result.result)))
            else:
                Logger.loginfo('[MavrosSetMode]: Mavros mode set to: {}'.format(self._request))

        except Exception as e:
            Logger.logerr('[MavrosSetMode]: Failed to call \'{}\' service request: {}'.format(self._service_topic, str(e)))
            self._failed = True
