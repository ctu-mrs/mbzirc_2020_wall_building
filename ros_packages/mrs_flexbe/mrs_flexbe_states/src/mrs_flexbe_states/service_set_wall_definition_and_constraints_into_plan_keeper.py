#!/usr/bin/env python
import rospy
import plan_keeper.srv

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 10/22/2019
@author: Robert Penicka
'''
class ServiceSetWallDefinitionAndConstraintsIntoPlanKeeper(EventState):
    '''
    State for saving the plans for robots in Wall challenge in MBZIRC 2020 into plan_keeper node.
        -- service_topic 	string      Service_topic_name.

        #> brick_rules                 Binary rules of bricks.
        #> brick_positions             Array of brick positions in the wall.
        #> brick_ids                   Array of brick ids ordered same as brick_positions.
        
        <= successed 		            Whenever the calling for successful.
        <= failed 		            When the calling failed.

    '''

    def __init__(self, plan_keeper_wall_definition_in_service_topic,plan_keeper_building_rules_in_service_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceSetWallDefinitionAndConstraintsIntoPlanKeeper, self).__init__(input_keys=['brick_rules','brick_positions','brick_ids'],
                                                           outcomes = ['successed', 'failed'])

        # Store state parameter for later use.
        self._plan_keeper_wall_definition_in_service_topic = rospy.resolve_name(plan_keeper_wall_definition_in_service_topic)
        self._plan_keeper_building_rules_in_service_topic = rospy.resolve_name(plan_keeper_building_rules_in_service_topic)
        
        # Create proxy service client
        self._srv_wall_definition = ProxyServiceCaller({self._plan_keeper_wall_definition_in_service_topic: plan_keeper.srv.SetWallDefinition})
        self._srv_building_rules = ProxyServiceCaller({self._plan_keeper_building_rules_in_service_topic: plan_keeper.srv.SetBuildingRules})
        
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

        # Check if the ProxyServiceCaller has been registered for setting wall definition
        if not self._srv_wall_definition.is_available(self._plan_keeper_wall_definition_in_service_topic):
            Logger.logerr('ProxyServiceCaller: Topic \'{}\' not yet registered!'.format(self._plan_keeper_wall_definition_in_service_topic))
            self._failed = True
            return

        try:

            service_request = plan_keeper.srv.SetWallDefinitionRequest()
            service_request.brick_positions = userdata.brick_positions
            service_request.brick_ids = userdata.brick_ids

            service_result = self._srv_wall_definition.call(self._plan_keeper_wall_definition_in_service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('Calling \'{}\' was not successful: {}'.format(self._plan_keeper_wall_definition_in_service_topic, str(service_result.message)))
            else:
                Logger.loginfo(service_result.message)

        except Exception as e:
            Logger.logerr('Failed to call \'{}\' service request: {}'.format(self._plan_keeper_wall_definition_in_service_topic, str(e)))
            self._failed = True
            
            
            
        # Check if the ProxyServiceCaller has been registered for setting constraints
        if not self._srv_building_rules.is_available(self._plan_keeper_building_rules_in_service_topic):
            Logger.logerr('ProxyServiceCaller: Topic \'{}\' not yet registered!'.format(self._plan_keeper_building_rules_in_service_topic))
            self._failed = True
            return

        try:

            service_request = plan_keeper.srv.SetBuildingRulesRequest()
            service_request.brick_rules = userdata.brick_rules
            
            service_result = self._srv_building_rules.call(self._plan_keeper_building_rules_in_service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('Calling \'{}\' was not successful: {}'.format(self._plan_keeper_building_rules_in_service_topic, str(service_result.message)))
            else:
                Logger.loginfo(service_result.message)

        except Exception as e:
            Logger.logerr('Failed to call \'{}\' service request: {}'.format(self._plan_keeper_building_rules_in_service_topic, str(e)))
            self._failed = True
