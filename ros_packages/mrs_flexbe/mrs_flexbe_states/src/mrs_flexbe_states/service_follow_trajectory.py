#!/usr/bin/env python
import rospy
import std_srvs.srv
import std_msgs.msg
import mrs_msgs.srv
import mrs_msgs.msg

import inspect
import rostopic
from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller
from flexbe_core.proxy.proxy_subscriber_cached import ProxySubscriberCached

'''
Created on 02/05/2020
@author: Robert Penicka
'''
class ServiceFollowTrajectory(EventState):
    '''
    State for start trajectory following 
        -- service_topic_follow 	       string      Service_topic_name to start following
        -- service_topic_set_trajectory    string      Service_topic_name to fill trajectory

        ># scanning_trajectory    Trajectory that is followed

        <= successed 		      Whenever the calling for successful.
        <= failed 		          When the calling failed.

    '''

    def __init__(self, service_topic_follow,service_topic_set_trajectory,control_manager_diagnostics_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceFollowTrajectory, self).__init__(input_keys=['scanning_trajectory','frame_id'],
                                                 outcomes = ['successed', 'failed'])


        # Store state parameter for later use.
        self._service_topic_follow = rospy.resolve_name(service_topic_follow)
        self._service_topic_set_trajectory = rospy.resolve_name(service_topic_set_trajectory)
        self._control_manager_diagnostics_topic = rospy.resolve_name(control_manager_diagnostics_topic)

        # Create proxy service client
        self._srv_follow = ProxyServiceCaller({self._service_topic_follow: std_srvs.srv.Trigger})
        self._srv_set_trajectory = ProxyServiceCaller({self._service_topic_set_trajectory: mrs_msgs.srv.TrajectoryReferenceSrv})

        (msg_path, msg_topic, fn) = rostopic.get_topic_type(self._control_manager_diagnostics_topic)

        if msg_topic == self._control_manager_diagnostics_topic:
            msg_type = self._get_msg_from_path(msg_path)
            self._sub_cont_diag = ProxySubscriberCached({self._control_manager_diagnostics_topic: msg_type})
            self._connected = True
            Logger.loginfo('[ServiceFollowTrajectory]: Successfully subscribed to topic %s' % self._control_manager_diagnostics_topic)

        self._failed = False


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.
        #Logger.logerr('[ServiceFollowTrajectory]: execute')
        
        if not self._connected:
            Logger.logerr('[ServiceFollowTrajectory]: not connected to %s' % self._control_manager_diagnostics_topic)
            return 'failed'

        diff_time = rospy.get_time() - self._start_time

        if self._sub_cont_diag.has_msg(self._control_manager_diagnostics_topic):
            rospy.loginfo_throttle(5,'[ServiceFollowTrajectory]: has diagnostics msg at time %s' % (str(rospy.get_time())))
            message = self._sub_cont_diag.get_last_msg(self._control_manager_diagnostics_topic)
            self._sub_cont_diag.remove_last_msg(self._control_manager_diagnostics_topic)
           
            if diff_time >= 1.0 and not message.tracker_status.tracking_trajectory:
                Logger.loginfo('[ServiceFollowTrajectory]: Successfully ended following of trajectory %s' % self._control_manager_diagnostics_topic)
                return 'successed'
            else:
                return

        
        if self._failed or diff_time >= len(userdata.scanning_trajectory) * 0.2 * 1.2:
            Logger.logerr('Failed follow trajectory \'{}\' within time {} s out of {} s'.format(self._service_topic_follow, str(diff_time),len(userdata.scanning_trajectory) * 0.2 * 1.2))
            return 'failed'
       


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.
        #Logger.logerr('[ServiceFollowTrajectory]: enter')
        self._start_time = rospy.get_time()
        if not self._connected:
            (msg_path, msg_topic, fn) = rostopic.get_topic_type(self._control_manager_diagnostics_topic)
            if msg_topic == self._control_manager_diagnostics_topic:
                msg_type = self._get_msg_from_path(msg_path)
                self._sub_cont_diag = ProxySubscriberCached({self._control_manager_diagnostics_topic: msg_type})
                self._connected = True
                Logger.loginfo('[ServiceFollowTrajectory]: Successfully subscribed to previously unavailable topic %s' % self._control_manager_diagnostics_topic)
            else:
                Logger.logwarn('[ServiceFollowTrajectory]: Topic %s still not available, giving up.' % self._control_manager_diagnostics_topic)

        if self._connected and self._sub_cont_diag.has_msg(self._control_manager_diagnostics_topic):
            Logger.loginfo('[ServiceFollowTrajectory]: Waiting for msg from topic %s' % self._control_manager_diagnostics_topic)
            self._sub_cont_diag.remove_last_msg(self._control_manager_diagnostics_topic)

        self._failed = False


        # Check if the ProxyServiceCaller has been registered
        if not self._srv_follow.is_available(self._service_topic_follow):
            Logger.logerr('[ServiceFollowTrajectory]: Topic \'{}\' not yet registered!'.format(self._service_topic_follow))
            self._failed = True
            return

        if not self._srv_set_trajectory.is_available(self._service_topic_set_trajectory):
            Logger.logerr('[ServiceFollowTrajectory]: Topic \'{}\' not yet registered!'.format(self._service_topic_set_trajectory))
            self._failed = True
            return

        try:
            service_request_set_trajectory = mrs_msgs.srv.TrajectoryReferenceSrvRequest()#TrackerTrajectorySrvRequest()
            trajectory_msg = mrs_msgs.msg.TrajectoryReference()
            trajectory_msg.header = std_msgs.msg.Header()
            trajectory_msg.header.stamp = rospy.Time.now()
            trajectory_msg.header.seq = 0
            trajectory_msg.header.frame_id = userdata.frame_id
            trajectory_msg.use_heading = True
            trajectory_msg.fly_now = True
            trajectory_msg.loop = False
            trajectory_msg.points = userdata.scanning_trajectory
            service_request_set_trajectory.trajectory = trajectory_msg

            service_result_set_trajectory = self._srv_set_trajectory.call(self._service_topic_set_trajectory,service_request_set_trajectory)

            Logger.logwarn('[ServiceFollowTrajectory]: Called \'{}\' in ServiceFollowTrajectory set trejectory'.format(self._service_topic_set_trajectory))
            if not service_result_set_trajectory.success:
                self._failed = True
                Logger.logwarn('[ServiceFollowTrajectory]: Calling \'{}\' was not successful'.format(self._service_topic_set_trajectory))
            else:
                Logger.loginfo('[ServiceFollowTrajectory]: Calling \'{}\' was successful'.format(self._service_topic_set_trajectory))


        except Exception as e:
            Logger.logerr('Failed to call \'{}\' service request: {}'.format(self._service_topic_follow, str(e)))
            self._failed = True
            
    def _get_msg_from_path(self, msg_path):
        msg_import = msg_path.split('/')
        msg_module = '%s.msg' % (msg_import[0])
        package = __import__(msg_module, fromlist=[msg_module])
        clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__.endswith(msg_import[1]))
        return clsmembers[0][1]
