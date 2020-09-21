#!/usr/bin/env python
import rospy
import plan_keeper.srv

import mbzirc_msgs.srv
from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 07/02/2020
@author: Robert Penicka
'''
class ServiceSetBrickDetectionTypeAndWallLayer(EventState):
    '''
    State for saving the plans for robots in Wall challenge in MBZIRC 2020 into plan_keeper node.
        -- service_topic_see_type   string      Service topic name for seting see WALL/BRICK..
        -- service_topic_layer      string      Service topic name for setting layer of wall
       
        ># see_type                            what to see
        ># wall_layer                          in which layer we are placing
        
        <= successed 		        Whenever the calling for successful.
        <= failed 		            When the calling failed.

    '''

    BRICK_SEE_EVERYTHING        = 0
    BRICK_SEE_RED               = 1
    BRICK_SEE_GREEN             = 2
    BRICK_SEE_BLUE              = 3
    BRICK_SEE_ORANGE            = 4
    BRICK_SEE_WALL              = 8
    BRICK_SEE_WALL_HAVING_RED   = 9
    BRICK_SEE_WALL_HAVING_GREEN = 10
    BRICK_SEE_WALL_HAVING_BLUE  = 11

    LAYER_0      			    = 0
    LAYER_1		                = 1

    def __init__(self, service_topic_see_type,service_topic_layer):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceSetBrickDetectionTypeAndWallLayer, self).__init__(input_keys=['see_type','wall_layer'],
                                                           outcomes = ['successed', 'failed'])

        # Store state parameter for later use.
        self._service_topic_see_type = rospy.resolve_name(service_topic_see_type)
        self._service_topic_layer = rospy.resolve_name(service_topic_layer)

        # Create proxy service client
        wait_time = 30
        rospy.loginfo("waiting for service "+self._service_topic_see_type)
        rospy.wait_for_service(self._service_topic_see_type, wait_time)
        self._srv_detection_type = ProxyServiceCaller({self._service_topic_see_type: mbzirc_msgs.srv.DetectionType})
        self._srv_layer = ProxyServiceCaller({self._service_topic_layer: mbzirc_msgs.srv.DetectionType})

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

        # Check if the ProxyServiceCaller has been registered
        if not self._srv_detection_type.is_available(self._service_topic_see_type):
            Logger.logerr('[ServiceSetBrickDetectionType]: Topic \'{}\' not yet registered!'.format(self._service_topic_see_type))
            self._failed = True
            return
        
        try:

            service_request_see_type = mbzirc_msgs.srv.DetectionTypeRequest()
            service_request_see_type.type = userdata.see_type
            Logger.loginfo('[ServiceSetBrickDetectionType]: setting see type %s'%(str(service_request_see_type.type)))
                           
            service_result_see_type = self._srv_detection_type.call(self._service_topic_see_type, service_request_see_type)

            if not service_result_see_type.success:
                self._failed = True
                Logger.logwarn('[ServiceSetBrickDetectionType]: Calling \'{}\' was not successful: {}'.format(self._service_topic_see_type, str(service_result_see_type.message)))
            else:
                Logger.loginfo(service_result_see_type.message)

        except Exception as e:
            Logger.logerr('[ServiceSetBrickDetectionType]:Failed to call \'{}\' service request: {}'.format(self._service_topic_see_type, str(e)))
            self._failed = True


        #part for setting layer
        if not self._srv_layer.is_available(self._service_topic_layer):
            Logger.logerr('[ServiceSetBrickDetectionType]: Topic \'{}\' not yet registered!'.format(self._service_topic_layer))
            self._failed = True
            return

        try:

            service_request_layer = mbzirc_msgs.srv.DetectionTypeRequest()
            service_request_layer.type = userdata.wall_layer
            Logger.loginfo('[ServiceSetBrickDetectionType]: setting slayer to %s'%(str(service_request_see_type.type)))
            
            service_result_layer = self._srv_layer.call(self._service_topic_layer, service_request_layer)

            if not service_result_layer.success:
                self._failed = True
                Logger.logwarn('[ServiceSetBrickDetectionType]: Calling \'{}\' was not successful: {}'.format(self._service_topic_layer, str(service_result_layer.message)))
            else:
                Logger.loginfo(service_result_layer.message)
        except Exception as e:
            Logger.logerr('[ServiceSetBrickDetectionType]:Failed to call \'{}\' service request: {}'.format(self._service_topic_layer, str(e)))
            self._failed = True


