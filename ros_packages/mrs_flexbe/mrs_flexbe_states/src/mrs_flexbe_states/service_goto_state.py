#!/usr/bin/env python
import rospy
import mrs_msgs.srv

import rostopic
import inspect
from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller
from flexbe_core.proxy.proxy_subscriber_cached import ProxySubscriberCached

'''
Created on 09/17/2019
@author: Vojtech Spurny, Robert Penicka
'''
class ServiceGoToState(EventState):
    '''
    State for calling goto service in MRS system.
        -- service_topic 	string                      Service_topic_name.

        ># goal                 mrs_msgs.msg.TrackerPoint    Target position [x,y,z,yaw].

        <= successed 		Whenever the calling for successful.
        <= failed 		When the calling failed.

    '''

    def __init__(self, service_topic,control_manager_diagnostics_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceGoToState, self).__init__(input_keys=['goal'],
                                              outcomes = ['successed', 'failed'])

        # Store state parameter for later use.
        self._service_topic = rospy.resolve_name(service_topic)
        self._control_manager_diagnostics_topic = rospy.resolve_name(control_manager_diagnostics_topic)

        # Create proxy service client
        self._srv = ProxyServiceCaller({self._service_topic: mrs_msgs.srv.Vec4})

        (msg_path, msg_topic, fn) = rostopic.get_topic_type(self._control_manager_diagnostics_topic)

        if msg_topic == self._control_manager_diagnostics_topic:
            msg_type = self._get_msg_from_path(msg_path)
            self._sub_cont_diag = ProxySubscriberCached({self._control_manager_diagnostics_topic: msg_type})
            self._connected = True
            Logger.loginfo('[ServiceGoToState]: Successfully subscribed to topic %s' % self._control_manager_diagnostics_topic)


        self._failed = False


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.
        if not self._connected:
            Logger.logerr('[ServiceGoToState]: not connected to %s' % self._control_manager_diagnostics_topic)
            return 'failed'
        
        diff_time = rospy.get_time() - self._start_time
        
        if self._sub_cont_diag.has_msg(self._control_manager_diagnostics_topic):
            Logger.loginfo('[ServiceGoToState]: has_diagnostics msg at time %s' % (str(rospy.get_time())))
            message = self._sub_cont_diag.get_last_msg(self._control_manager_diagnostics_topic)
            self._sub_cont_diag.remove_last_msg(self._control_manager_diagnostics_topic)
           
            if diff_time >= 1.0 and not message.tracker_status.have_goal:
                Logger.loginfo('[ServiceGoToState]: Successfully ended following of trajectory %s' % self._control_manager_diagnostics_topic)
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
                Logger.loginfo('[ServiceGoToState]: Successfully subscribed to previously unavailable topic %s' % self._control_manager_diagnostics_topic)
            else:
                Logger.logwarn('[ServiceGoToState]: Topic %s still not available, giving up.' % self._control_manager_diagnostics_topic)

        if self._connected and self._sub_cont_diag.has_msg(self._control_manager_diagnostics_topic):
            Logger.loginfo('[ServiceGoToState]: Waiting for msg from topic %s' % self._control_manager_diagnostics_topic)
            self._sub_cont_diag.remove_last_msg(self._control_manager_diagnostics_topic)

        
        self._failed = False

        # Check if the ProxyServiceCaller has been registered
        if not self._srv.is_available(self._service_topic):
            Logger.logerr('[ServiceGoToState]: Topic \'{}\' not yet registered!'.format(self._service_topic))
            self._failed = True
            return

        try:
            service_request = mrs_msgs.srv.Vec4Request()
            service_request.goal[0] = userdata.goal.x # x
            service_request.goal[1] = userdata.goal.y # y
            service_request.goal[2] = userdata.goal.z # z
            service_request.goal[3] = userdata.goal.yaw # yaw
            
            Logger.loginfo('[ServiceGoToState]: calling goto %f %f %f %f'%(userdata.goal.x,userdata.goal.y,userdata.goal.z,userdata.goal.yaw))
            service_result = self._srv.call(self._service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('[ServiceGoToState]: Calling \'{}\' was not successful: {}'.format(self._service_topic, str(service_result.message)))
            else:
                Logger.loginfo(service_result.message)

        except Exception as e:
            Logger.logerr('[ServiceGoToState]: Failed to call \'{}\' service request: {}'.format(self._service_topic, str(e)))
            self._failed = True
            
    def _get_msg_from_path(self, msg_path):
        msg_import = msg_path.split('/')
        msg_module = '%s.msg' % (msg_import[0])
        package = __import__(msg_module, fromlist=[msg_module])
        clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__.endswith(msg_import[1]))
        return clsmembers[0][1]
