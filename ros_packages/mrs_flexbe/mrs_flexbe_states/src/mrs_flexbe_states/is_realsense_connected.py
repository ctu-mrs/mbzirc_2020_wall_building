#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller
from flexbe_core.proxy.proxy_subscriber_cached import ProxySubscriberCached
import sensor_msgs.msg

import inspect
import rostopic

class IsRealsenseConnectedState(EventState):
    '''
    Checking if realsense is connected

    -- battery_topic     string     topic of realsense image info

    <= continue             Given time has passed.
    <= failed                 Example for a failure outcome.

    '''

    def __init__(self, realsense_camera_info_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(IsRealsenseConnectedState, self).__init__(input_keys=['max_time_unconnected_realsense_s'],
                                                        outcomes=['is_connected', 'unconnected']
                                                    )

        # Store state parameter for later use.
        self.realsense_camera_info_topic = rospy.resolve_name(realsense_camera_info_topic)
        
        (msg_path, msg_topic, fn) = rostopic.get_topic_type(self.realsense_camera_info_topic)
        if msg_topic == self.realsense_camera_info_topic:
            msg_type = self._get_msg_from_path(msg_path)
            self.realsense_subs_ = ProxySubscriberCached({self.realsense_camera_info_topic: msg_type})
            self._connected = True
            Logger.loginfo('[IsRealsenseConnectedState]: Successfully subscribed to topic %s' % self.realsense_camera_info_topic)
            #realsense_subs_ = rospy.Subscriber(self.realsense_camera_info_topic, msg_type, self.realsense_image_info_callback)

        self.max_non_connected_time_s = 3.0
        self.last_time = rospy.get_time()
        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        

    def realsense_image_info_callback(self, data):
        self.last_time = rospy.get_time()
        rospy.loginfo_throttle(20,'[IsRealsenseConnectedState] realsense still connected')

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.
        self.max_non_connected_time_s = userdata.max_time_unconnected_realsense_s
        
        if self.realsense_subs_.has_msg(self.realsense_camera_info_topic):
            self.last_time = rospy.get_time()
            Logger.loginfo('[IsRealsenseConnectedState] realsense is connected')
            return 'is_connected'
        
        diff_time = rospy.get_time() - self.last_time
        if diff_time > self.max_non_connected_time_s:
            Logger.logerr('[IsRealsenseConnectedState] realsense ')
            return 'unconnected'
        
        
            
    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        Logger.loginfo('[IsRealsenseConnectedState] entering state')
        self.last_time = rospy.get_time()
        
        if not self._connected:
            (msg_path, msg_topic, fn) = rostopic.get_topic_type(self.realsense_camera_info_topic)
            if msg_topic == self.realsense_camera_info_topic:
                msg_type = self._get_msg_from_path(msg_path)
                self.realsense_subs_ = ProxySubscriberCached({self.realsense_camera_info_topic: msg_type})
                self._connected = True
                Logger.loginfo('[IsRealsenseConnectedState]: Successfully subscribed to previously unavailable topic %s' % self.realsense_camera_info_topic)
            else:
                Logger.logwarn('[IsRealsenseConnectedState]: Topic %s still not available, giving up.' % self.realsense_camera_info_topic)
                
        if self._connected and self.realsense_subs_.has_msg(self.realsense_camera_info_topic):
            Logger.loginfo('[IsRealsenseConnectedState]: Waiting for msg from topic %s' % self.realsense_camera_info_topic)
            self.realsense_subs_.remove_last_msg(self.realsense_camera_info_topic)
    
    def _get_msg_from_path(self, msg_path):
		msg_import = msg_path.split('/')
		msg_module = '%s.msg' % (msg_import[0])
		package = __import__(msg_module, fromlist=[msg_module])
		clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__.endswith(msg_import[1]))
		return clsmembers[0][1]    
