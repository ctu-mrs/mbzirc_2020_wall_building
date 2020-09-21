#!/usr/bin/env python
import rospy
import mrs_msgs.srv

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 08/20/2019
@author: Vojtech Spurny
'''
class ServiceStringState(EventState):
    '''
    State for calling std_srvs/Trigger type of services.
        -- service_topic 	string      Service_topic_name.
        -- state_name 	        string 	    Name of the state used during printing.
        -- msg_text 	        string 	    text msg.

        <= finished 		Whenever the calling for successful.
        <= failed 		When the calling failed.

    '''

    def __init__(self, service_topic, msg_text, state_name = None):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceStringState, self).__init__(outcomes = ['finished', 'failed'])

        # Store state parameter for later use.
        self._service_topic = rospy.resolve_name(service_topic)
        if state_name is None:
            self._state_name = ""
        else:
            self._state_name = '[{}]: '.format(state_name) 

        # Create proxy service client
        self._srv = ProxyServiceCaller({self._service_topic: mrs_msgs.srv.String})
        self._msg_text = msg_text

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
            Logger.logerr('{}ProxyServiceCaller: Topic \'{}\' not yet registered!'.format(self._state_name, self._service_topic))
            self._failed = True
            return

        try:
            service_request = mrs_msgs.srv.StringRequest()
            service_request.value = self._msg_text
            service_result = self._srv.call(self._service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('{}Calling \'{}\' was not successful: {}'.format(self._state_name, self._service_topic, str(service_result.message)))
            else:
                Logger.loginfo(self._state_name + str(service_result.message))

        except Exception as e:
            Logger.logerr('{}Failed to call \'{}\' service request: {}'.format(self._state_name, self._service_topic, str(e)))
            self._failed = True
