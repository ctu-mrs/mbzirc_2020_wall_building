#!/usr/bin/env python
import rospy
import rostopic
import inspect
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached

'''
Created on 08/21/2019
@author: Vojtech Spurny
'''

class WaitForMsgState(EventState):
    '''
    Check new messages on the given topic if they are satisfying the given function.
    Whenever the function returns True the outcome will be 'successed'.
    If the function have not been True during the specified time the outcome will be 'failed'.
    Only message that are received since this state is active are processed.

        -- topic 		string		The topic on which should be listened.
        -- wait_time 	        float	        Amount of time to wait for the function to be valid.
        -- function     	 		Function that check the comming msgs. The function should return bool value
        -- input_keys     	string[] 	State machine variables that will are in the input.
        -- output_keys     	string[] 	State machine variables that will be returned.
        -- output_function     	     	        Function that is called whenever calling function with incoming message was successful.
                                                It should be used in the case you want to store output_key with message

        <= successed 				The function processed a msg with valid result.
        <= failed 				Timeout has been reached before the function was valid.

    '''

    @staticmethod
    def default_output_function(message, userdata):
        pass

    def __init__(self, topic, wait_time, function, input_keys = [], output_keys = [], output_function = None):
        '''
        Constructor
        '''
        super(WaitForMsgState, self).__init__(outcomes=['successed', 'failed'], output_keys = output_keys, input_keys = input_keys)

        self._topic = rospy.resolve_name(topic)
        self._wait_time = wait_time
        self._function = function
        self._connected = False
        if output_function is None:
            self._output_function = WaitForMsgState.default_output_function
        else:
            self._output_function = output_function

        (msg_path, msg_topic, fn) = rostopic.get_topic_type(self._topic)

        if msg_topic == self._topic:
            msg_type = self._get_msg_from_path(msg_path)
            self._sub = ProxySubscriberCached({self._topic: msg_type})
            self._connected = True
        else:
            Logger.logwarn('[WaitForMsg]: Topic %s for state %s not yet available.\nFound: %s\nWill try again when entering the state...' % (self._topic, self.name, str(msg_topic)))


    def execute(self, userdata):
        '''
        Execute this state
        '''
        if not self._connected:
            return 'failed'

        if self._sub.has_msg(self._topic):
            message = self._sub.get_last_msg(self._topic)
            self._sub.remove_last_msg(self._topic)
            try:
                if self._function(message):
                    self._output_function(message, userdata)
                    return 'successed'
                else:
                    return

            except Exception as e:
                Logger.logerr('[WaitForMsg]: Failed to use function. Error is: %s' % str(e))
                return 'failed'

        if self._wait_time >= 0:
            elapsed = rospy.get_rostime() - self._start_time;
            if (elapsed.to_sec() > self._wait_time):
                Logger.logerr('[WaitForMsg]: Condition was not satisfied in time limit %d s!'% self._wait_time)
                return 'failed'


    def on_enter(self, userdata):
        self._start_time = rospy.get_rostime()
        if not self._connected:
            (msg_path, msg_topic, fn) = rostopic.get_topic_type(self._topic)
            if msg_topic == self._topic:
                msg_type = self._get_msg_from_path(msg_path)
                self._sub = ProxySubscriberCached({self._topic: msg_type})
                self._connected = True
                Logger.loginfo('[WaitForMsg]: Successfully subscribed to previously unavailable topic %s' % self._topic)
            else:
                Logger.logwarn('[WaitForMsg]: Topic %s still not available, giving up.' % self._topic)

            # removed last msg
        if self._connected and self._sub.has_msg(self._topic):
            Logger.loginfo('[WaitForMsg]: Waiting for msg from topic %s' % self._topic)
            self._sub.remove_last_msg(self._topic)


    def _get_msg_from_path(self, msg_path):
        msg_import = msg_path.split('/')
        msg_module = '%s.msg' % (msg_import[0])
        package = __import__(msg_module, fromlist=[msg_module])
        clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__.endswith(msg_import[1]))
        return clsmembers[0][1]
