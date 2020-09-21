#!/usr/bin/env python
import rospy
import rostopic
import inspect
from flexbe_core import EventState, Logger


from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller
from flexbe_core.proxy.proxy_subscriber_cached import ProxySubscriberCached
import brick_mapping.msg 

'''
Created on 08/21/2019
@author: Vojtech Spurny
'''

class WaitForArenaMappedObjectsState(EventState):
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

    def __init__(self, plan_keeper_mapped_arena_topic):
        '''
        Constructor
        '''
        super(WaitForArenaMappedObjectsState, self).__init__(outcomes=['successed', 'failed'], output_keys = ['mapped_objects'])

        self._topic = rospy.resolve_name(plan_keeper_mapped_arena_topic)
  
        #self._sub = rospy.Subscriber(self._topic, brick_mapping.msg.MappedArenaObjectsStamped, self.mapped_area_callback)
        #self._connected = True
        #
        (msg_path, msg_topic, fn) = rostopic.get_topic_type(self._topic)
        #Logger.loginfo('[IsBatteryBellowValueState]: topic %s msg_topic %s'%(self._topic,msg_topic))
        if msg_topic == self._topic:
            msg_type = self._get_msg_from_path(msg_path)
            self._sub = ProxySubscriberCached({self._topic: msg_type})
            self._connected = True
            Logger.loginfo('[WaitForArenaMappedObjectsState]: subscribing topic %s ' % (self._topic))
        
        self.received = False
        self.mapped_objects = None

    #def mapped_area_callback(self,message):
    #    rospy.loginfo_throttle(5,"[WaitForArenaMappedObjectsState]: received mapped objects")
    #    self.mapped_objects = message.mapped_objects
    #    self.received = True

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self._sub_battery.has_msg(self._topic): 
            message = self._sub_battery.get_last_msg(self._topic)
            self._sub_battery.remove_last_msg(self._topic)   
        #if self.received:
            userdata.mapped_objects = message.mapped_objects
            rospy.logwarn("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            rospy.logwarn("[WaitForArenaMappedObjectsState]: received mapped objects")
            rospy.logwarn("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            return 'successed'
        else:
            rospy.loginfo_throttle(5,"[WaitForArenaMappedObjectsState]: waiting for mapped objects")
            

    def on_enter(self, userdata):
        Logger.loginfo('[WaitForArenaMappedObjectsState]: entering state %s' % self._topic)
        
        if not self._connected:
            (msg_path, msg_topic, fn) = rostopic.get_topic_type(self._topic)
            #Logger.loginfo('[IsBatteryBellowValueState]: topic %s msg_topic %s'%(self._topic,msg_topic))
            if msg_topic == self._topic:
                msg_type = self._get_msg_from_path(msg_path)
                self._sub = ProxySubscriberCached({self._topic: msg_type})
                self._connected = True
                Logger.loginfo('[WaitForArenaMappedObjectsState]: subscribing topic %s ' % (self._topic))
            else:
                Logger.logwarn('[WaitForArenaMappedObjectsState]: Topic %s still not available, giving up.' % self._topic)
   
    def _get_msg_from_path(self, msg_path):
        msg_import = msg_path.split('/')
        msg_module = '%s.msg' % (msg_import[0])
        package = __import__(msg_module, fromlist=[msg_module])
        clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__.endswith(msg_import[1]))
        return clsmembers[0][1]
