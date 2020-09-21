#!/usr/bin/env python
import rospy
import ctop_planner.srv

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 10/22/2019
@author: Robert Penicka
'''
class ServiceGetWallDefinitionAndConstraints(EventState):
    '''
    State for getting wall definition and brick rules for robots in Wall challenge in MBZIRC 2020.
        -- service_topic 	string      Service_topic_name.

        #> brick_rules                 Binary rules of bricks.
        #> brick_positions             Array of brick positions in the wall.
        #> brick_ids                   Array of brick ids ordered same as brick_positions.
        

        <= successed 		            Whenever the calling for successful.
        <= failed 		            When the calling failed.

    '''

    def __init__(self, wall_definition_service_topic, building_rules_service_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceGetWallDefinitionAndConstraints, self).__init__(input_keys=[],
                                                 output_keys=['brick_rules','brick_positions','brick_ids'],
                                                 outcomes = ['successed', 'failed'])

        # Store state parameter for later use.
        self._wall_definition_service_topic = rospy.resolve_name(wall_definition_service_topic)
        self._building_rules_service_topic = rospy.resolve_name(building_rules_service_topic)

        # Create proxy service client
        self._srv_wall_definition = ProxyServiceCaller({self._wall_definition_service_topic: ctop_planner.srv.WallDefinition})
        self._srv_building_rules = ProxyServiceCaller({self._building_rules_service_topic: ctop_planner.srv.BuildingRules})

        self._failed = False


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._failed:
            return 'failed'
        else:
            return 'successed'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        self._failed = False

        # Check if the ProxyServiceCaller has been registered for wall definition
        if not self._srv_wall_definition.is_available(self._wall_definition_service_topic):
            Logger.logerr('ProxyServiceCaller: Topic \'{}\' not yet registered!'.format(self._wall_definition_service_topic))
            self._failed = True
            return

        try:

            service_request = ctop_planner.srv.WallDefinitionRequest()
            service_result = self._srv_wall_definition.call(self._wall_definition_service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('Calling \'{}\' was not successful: {}'.format(self._wall_definition_service_topic, str(service_result.message)))
            else:
                Logger.loginfo(service_result.message)
                userdata.brick_positions = service_result.brick_positions
                userdata.brick_ids = service_result.brick_ids

        except Exception as e:
            Logger.logerr('Failed to call \'{}\' service request: {}'.format(self._wall_definition_service_topic, str(e)))
            self._failed = True
            
            
        # Check if the ProxyServiceCaller has been registered for brick rules
        if not self._srv_building_rules.is_available(self._building_rules_service_topic):
            Logger.logerr('ProxyServiceCaller: Topic \'{}\' not yet registered!'.format(self._building_rules_service_topic))
            self._failed = True
            return

        try:

            service_request = ctop_planner.srv.BuildingRulesRequest()
            service_result = self._srv_building_rules.call(self._building_rules_service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('Calling \'{}\' was not successful: {}'.format(self._building_rules_service_topic, str(service_result.message)))
            else:
                Logger.loginfo(service_result.message)
                userdata.brick_rules = service_result.brick_rules

        except Exception as e:
            Logger.logerr('Failed to call \'{}\' service request: {}'.format(self._building_rules_service_topic, str(e)))
            self._failed = True
            
            
            
