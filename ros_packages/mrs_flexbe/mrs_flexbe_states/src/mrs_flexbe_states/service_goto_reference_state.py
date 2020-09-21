#!/usr/bin/env python
import rospy
import mrs_msgs.srv
import mrs_msgs.msg
import std_msgs.msg

import inspect
from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller
from flexbe_core.proxy.proxy_subscriber_cached import ProxySubscriberCached

import rostopic

'''
Created on 09/17/2019
@author: Vojtech Spurny, Robert Penicka
'''


class ServiceGoToReferenceState(EventState):
    '''
    State for calling goto service in MRS system.
        -- service_topic 	string                                      Service_topic_name.

        ># goal_tracker_point       mrs_msgs.srv.ReferenceStampedSrvRequest    Target position in Reference style 
        ># frame_id                 string                                     frame id of targetreference

        <= successed 		Whenever the calling for successful.
        <= failed 		When the calling failed.

    '''

    def __init__(self, service_topic,control_manager_diagnostics_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceGoToReferenceState, self).__init__(input_keys=['goal_tracker_point', 'frame_id'],
                                              outcomes=['successed', 'failed'])

        # Store state parameter for later use.
        self._service_topic = rospy.resolve_name(service_topic)
        self._control_manager_diagnostics_topic = rospy.resolve_name(control_manager_diagnostics_topic)

        # Create proxy service client
        self._srv = ProxyServiceCaller({self._service_topic: mrs_msgs.srv.ReferenceStampedSrv})
        self.target = None
        (msg_path, msg_topic, fn) = rostopic.get_topic_type(self._control_manager_diagnostics_topic)

        if msg_topic == self._control_manager_diagnostics_topic:
            msg_type = self._get_msg_from_path(msg_path)
            self._sub_cont_diag = ProxySubscriberCached({self._control_manager_diagnostics_topic: msg_type})
            self._connected = True
            Logger.loginfo('[GoToReference]: Successfully subscribed to topic %s' % self._control_manager_diagnostics_topic)

        self._failed = False

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if not self._connected:
            Logger.logerr('[GoToReference]: not connected to %s' % self._control_manager_diagnostics_topic)
            return 'failed'
        
        diff_time = rospy.get_time() - self._start_time
        
        if self._sub_cont_diag.has_msg(self._control_manager_diagnostics_topic):
            #Logger.loginfo('[GoToReference]: has_diagnostics msg at time %s' % (str(rospy.get_time())))
            message = self._sub_cont_diag.get_last_msg(self._control_manager_diagnostics_topic)
            self._sub_cont_diag.remove_last_msg(self._control_manager_diagnostics_topic)
           
            if diff_time >= 1.0 and not message.tracker_status.have_goal:
                Logger.loginfo('[GoToReference]: Successfully ended following of trajectory %s' % self._control_manager_diagnostics_topic)
                return 'successed'
            else:
                rospy.loginfo_throttle(2,'[GoToReference]: still tracking diff time %.2f to ref x%.2f y%.2f z%.2f head%.2f'%(diff_time,self.target.position.x,self.target.position.y,self.target.position.z,self.target.heading))
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
                Logger.loginfo('[GoToReference]: Successfully subscribed to previously unavailable topic %s' % self._control_manager_diagnostics_topic)
            else:
                Logger.logwarn('[GoToReference]: Topic %s still not available, giving up.' % self._control_manager_diagnostics_topic)

        if self._connected and self._sub_cont_diag.has_msg(self._control_manager_diagnostics_topic):
            Logger.loginfo('[GoToReference]: Waiting for msg from topic %s' % self._control_manager_diagnostics_topic)
            self._sub_cont_diag.remove_last_msg(self._control_manager_diagnostics_topic)
            
        self._failed = False

        # Check if the ProxyServiceCaller has been registered
        if not self._srv.is_available(self._service_topic):
            Logger.logerr('[GoToReference]: Topic \'{}\' not yet registered!'.format(self._service_topic))
            self._failed = True
            return

        try:
            service_request = mrs_msgs.srv.ReferenceStampedSrvRequest()
            service_request.header = std_msgs.msg.Header()
            service_request.header.stamp = rospy.Time.now()
            service_request.header.seq = 0
            service_request.header.frame_id = userdata.frame_id
            service_request.reference.position.x = userdata.goal_tracker_point.position.x
            service_request.reference.position.y = userdata.goal_tracker_point.position.y
            service_request.reference.position.z = userdata.goal_tracker_point.position.z
            service_request.reference.heading = userdata.goal_tracker_point.heading
            self.target = service_request.reference
            Logger.loginfo('[GoToReference]: called to ref x%.2f y%.2f z%.2f head%.2f'%(service_request.reference.position.x,service_request.reference.position.y,service_request.reference.position.z,service_request.reference.heading))
            service_result = self._srv.call(self._service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('[GoToReference]: Calling \'{}\' was not successful: {} for reference position (x,y,z) = ({},{},{})'.format(self._service_topic, str(service_result.message),service_request.reference.position.x,service_request.reference.position.y,service_request.reference.position.z))
            else:
                Logger.loginfo('[GoToReference]: {}'.format(str(service_result.message)))

        except Exception as e:
            Logger.logerr('[GoToReference]: Failed to call \'{}\' service request: {}'.format(self._service_topic, str(e)))
            self._failed = True
            
    def _get_msg_from_path(self, msg_path):
        msg_import = msg_path.split('/')
        msg_module = '%s.msg' % (msg_import[0])
        package = __import__(msg_module, fromlist=[msg_module])
        clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__.endswith(msg_import[1]))
        return clsmembers[0][1]
