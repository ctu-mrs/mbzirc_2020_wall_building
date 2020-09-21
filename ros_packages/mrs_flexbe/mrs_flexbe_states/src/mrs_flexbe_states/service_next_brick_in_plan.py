#!/usr/bin/env python
import rospy
import plan_keeper.srv
import copy

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 09/25/2019
@author: Vojtech Spurny
'''


class ServiceGetNextBrickInPlan(EventState):
    '''
    State for getting a plan for next brick in Wall challenge in MBZIRC 2020.
        -- service_topic 	string      Service_topic_name.
        -- flying_altitude 	float       Altitude when the robot will move.

        #> grasp_position       TrackerPoint  Position with bricks.
        #> placing_position        TrackerPoint  Position where to place brick after grasping.
        #> wall_position        TrackerPoint  Position where the robot should go before calling dropping.
        #> brick_type           int8          Type of the brick.
        #> brick_id             int16         Id of the brick.

        <= successed 		            Whenever the calling for successful.
        <= failed 		                When the calling failed.
        <= finished                     When plan is finished

    '''

    def __init__(self, service_topic, flying_altitude ):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceGetNextBrickInPlan, self).__init__(output_keys=['grasp_position', 'placing_position', 'drop_wait_position', 'next_to_wall_wait_position', 'brick_type','brick_see_wall_typed', 'brick_id','wall_layer'],
                                                        outcomes=['successed', 'finished', 'failed'])

        # Store state parameter for later use.
        self._service_topic = rospy.resolve_name(service_topic)
        self._robot_name = rospy.get_namespace()[1:-1]  # removing backslash from beggining and end
        self._flying_altitude = flying_altitude
        

        # Create proxy service client
        self._srv = ProxyServiceCaller({self._service_topic: plan_keeper.srv.BrickPlan})

        self._failed = False
        self._finished = False

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._finished:
        	return 'finished'

        if self._failed:
            return 'failed'
        else:
            return 'successed'

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        self._failed = False
        self._finished = False

        # Check if the ProxyServiceCaller has been registered
        if not self._srv.is_available(self._service_topic):
            Logger.logerr('ProxyServiceCaller: Topic \'{}\' not yet registered!'.format(self._service_topic))
            self._failed = True
            return

        try:

            service_request = plan_keeper.srv.BrickPlanRequest()
            service_request.robot_name = self._robot_name

            service_result = self._srv.call(self._service_topic, service_request)

            self._finished = service_result.finished

            if not service_result.success:
                self._failed = True
                Logger.logwarn('Calling \'{}\' was not successful: {}'.format(self._service_topic, str(service_result.message)))
            else:
                Logger.loginfo('[ServiceGetNextBrickInPlan]: {}'.format(service_result.message))
                service_result.grasp_position.position.z = self._flying_altitude
                userdata.grasp_position = service_result.grasp_position
                userdata.placing_position = service_result.drop_position
                userdata.brick_type = service_result.brick_type
                userdata.brick_id = service_result.brick_id
                userdata.wall_layer = service_result.wall_layer
                userdata.brick_see_wall_typed = service_result.brick_see_wall_typed
                userdata.next_to_wall_wait_position = service_result.next_to_wall_wait_position
                
                drop_wait_position = copy.deepcopy(service_result.drop_wait_position)
                drop_wait_position.position.z = self._flying_altitude
                userdata.drop_wait_position = drop_wait_position
                

        except Exception as e:
            Logger.logerr('Failed to call \'{}\' service request: {}'.format(self._service_topic, str(e)))
            self._failed = True
