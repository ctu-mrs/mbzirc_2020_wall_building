#!/usr/bin/env python
import rospy
import mrs_msgs.srv
import mrs_msgs.msg
import std_msgs.msg

import inspect
from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller
from flexbe_core.proxy.proxy_subscriber_cached import ProxySubscriberCached
import tf.transformations
import math

import rostopic

'''
Created on 10/02/2020
@author: Robert Penicka
'''


class ServiceRotateRelativeToGoToState(EventState):
    '''
    State for calling goto service in MRS system.
        -- set_yaw_service_topic 	string                        service topic for setting yaw.
        -- control_manager_diagnostics_topic 	string            service topic for control diagnostics.

        ># goal_tracker_point               mrs_msgs.srv.ReferenceStampedSrvRequest    Target position in Reference style 

        <= successed 		Whenever the calling for successful.
        <= failed 		When the calling failed.

    '''

    def __init__(self, set_yaw_service_topic,control_manager_diagnostics_topic,odometry_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceRotateRelativeToGoToState, self).__init__(input_keys=['goal_tracker_point', 'frame_id','yaw_relative_to_goal'],
                                                               output_keys=['desired_yaw'],
                                                               outcomes=['successed', 'failed'])

        # Store state parameter for later use.
        self._set_yaw_service_topic = rospy.resolve_name(set_yaw_service_topic)
        self._control_manager_diagnostics_topic = rospy.resolve_name(control_manager_diagnostics_topic)
        self._odometry_topic = rospy.resolve_name(odometry_topic)
        
        # Create proxy service client
        self._srv = ProxyServiceCaller({self._set_yaw_service_topic: mrs_msgs.srv.Vec1})

        (msg_path, msg_topic, fn) = rostopic.get_topic_type(self._control_manager_diagnostics_topic)

        if msg_topic == self._control_manager_diagnostics_topic:
            msg_type = self._get_msg_from_path(msg_path)
            self._sub_cont_diag = ProxySubscriberCached({self._control_manager_diagnostics_topic: msg_type})
            self._connected = True
            Logger.loginfo('[ServiceRotateRelativeToGoToState]: Successfully subscribed to topic %s' % self._control_manager_diagnostics_topic)
        
        (msg_path_odom, msg_topic_odom, fn_odom) = rostopic.get_topic_type(self._odometry_topic)
        if msg_topic_odom == self._odometry_topic:
            msg_type_odom = self._get_msg_from_path(msg_path_odom)
            self._sub_odom = ProxySubscriberCached({self._odometry_topic: msg_type_odom})
            self._connected_odom = True
            Logger.loginfo('[ServiceRotateRelativeToGoToState]: Successfully subscribed to topic %s' % self._odometry_topic)
            
        self._desired_yaw = 0
        self._sended = False
        self._failed = False

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if not self._connected:
            Logger.logerr('[ServiceRotateRelativeToGoToState]: not connected to %s' % self._control_manager_diagnostics_topic)
            return 'failed'
        
        if not self._connected_odom:
            Logger.logerr('[ServiceRotateRelativeToGoToState]: not connected to %s' % self._odometry_topic)
            return 'failed'
        
        diff_time = rospy.get_time() - self._start_time
        
        if self._sub_cont_diag.has_msg(self._control_manager_diagnostics_topic) and self._sub_odom.has_msg(self._odometry_topic):
            Logger.loginfo('[ServiceRotateRelativeToGoToState]: has_diagnostics msg at time %s' % (str(diff_time)))
            message = self._sub_cont_diag.get_last_msg(self._control_manager_diagnostics_topic)
            message_odom = self._sub_odom.get_last_msg(self._odometry_topic)
            
            currentPosition = message_odom.pose.pose.position
            target_position = userdata.goal_tracker_point.position
            self._desired_yaw = math.atan2(target_position.y - currentPosition.y,target_position.x - currentPosition.x)
            self._sub_cont_diag.remove_last_msg(self._control_manager_diagnostics_topic)
            self._sub_odom.remove_last_msg(self._odometry_topic)
            
            rospy.loginfo_throttle(2,'[ServiceRotateRelativeToGoToState]: currentPosition %s %s' % (str(currentPosition.x),str(currentPosition.y)))
            rospy.loginfo_throttle(2,'[ServiceRotateRelativeToGoToState]: target_position %s %s' % (str(target_position.x),str(target_position.y)))
            rospy.loginfo_throttle(2,'[ServiceRotateRelativeToGoToState]: self._desired_yaw %s' % (str(self._desired_yaw)))
           
            if not self._sended:
                
                # Check if the ProxyServiceCaller has been registered
                if not self._srv.is_available(self._set_yaw_service_topic):
                    Logger.logerr('[ServiceRotateRelativeToGoToState]: Topic \'{}\' not yet registered!'.format(self._set_yaw_service_topic))
                    self._failed = True
                    return
        
                try:
                    service_request = mrs_msgs.srv.Vec1Request()
                    service_request.goal = self._desired_yaw
                    service_result = self._srv.call(self._set_yaw_service_topic, service_request)
                    Logger.loginfo('[ServiceRotateRelativeToGoToState]: sended desired yaw')
                    if not service_result.success:
                        self._failed = True
                        Logger.logwarn('[ServiceRotateRelativeToGoToState]: Calling \'{}\' was not successful: {} for reference position (x,y,z) = ({},{},{})'.format(self._service_topic, str(service_result.message),service_request.goal))
                    else:
                        self._sended = True
                        self._start_time = rospy.get_time()
                        userdata.desired_yaw = self._desired_yaw
                        Logger.loginfo('[ServiceRotateRelativeToGoToState]: call result {}'.format(str(service_result.message)))
        
                except Exception as e:
                    Logger.logerr('[ServiceRotateRelativeToGoToState]: Failed to call \'{}\' service request: {}'.format(self._set_yaw_service_topic, str(e)))
                    self._failed = True
                    
            diff_time = rospy.get_time() - self._start_time
            
           
            if diff_time >= 1.0 and not message.tracker_status.have_goal and self._sended:
                Logger.loginfo('[ServiceRotateRelativeToGoToState]: Successfully ended following of trajectory %s' % self._control_manager_diagnostics_topic)
                return 'successed'
            else:
                return
            
        if self._failed:
            return 'failed'

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.
          
        self._start_time = rospy.get_time()
        
        if not self._connected:
            (msg_path, msg_topic, fn) = rostopic.get_topic_type(self._control_manager_diagnostics_topic)
            if msg_topic == self._control_manager_diagnostics_topic:
                msg_type = self._get_msg_from_path(msg_path)
                self._sub_cont_diag = ProxySubscriberCached({self._control_manager_diagnostics_topic: msg_type})
                self._connected = True
                Logger.loginfo('[ServiceRotateRelativeToGoToState]: Successfully subscribed to previously unavailable topic %s' % self._control_manager_diagnostics_topic)
            else:
                Logger.logwarn('[ServiceRotateRelativeToGoToState]: Topic %s still not available, giving up.' % self._control_manager_diagnostics_topic)
        
        if not self._connected_odom:
            (msg_path_odom, msg_topic_odom, fn_odom) = rostopic.get_topic_type(self._odometry_topic)
            if msg_topic_odom == self._odometry_topic:
                msg_type_odom = self._get_msg_from_path(msg_path_odom)
                self._sub_odom = ProxySubscriberCached({self._odometry_topic: msg_type_odom})
                self._connected_odom = True
                Logger.loginfo('[ServiceRotateRelativeToGoToState]: Successfully subscribed to topic %s' % self._odometry_topic)
            else:
                Logger.logwarn('[ServiceRotateRelativeToGoToState]: Topic %s still not available, giving up.' % self._odometry_topic)
            
        
        if self._connected and self._sub_cont_diag.has_msg(self._control_manager_diagnostics_topic):
            Logger.loginfo('[ServiceRotateRelativeToGoToState]: Waiting for msg from topic %s' % self._control_manager_diagnostics_topic)
            self._sub_cont_diag.remove_last_msg(self._control_manager_diagnostics_topic)
            
        if self._connected_odom and self._sub_odom.has_msg(self._odometry_topic):
            Logger.loginfo('[ServiceRotateRelativeToGoToState]: Waiting for msg from topic %s' % self._odometry_topic)
            self._sub_odom.remove_last_msg(self._odometry_topic)
            
        self._failed = False

    def _get_msg_from_path(self, msg_path):
        msg_import = msg_path.split('/')
        msg_module = '%s.msg' % (msg_import[0])
        package = __import__(msg_module, fromlist=[msg_module])
        clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__.endswith(msg_import[1]))
        return clsmembers[0][1]
