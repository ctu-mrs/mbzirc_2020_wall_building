#!/usr/bin/env python
import rospy
import plan_keeper.srv

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 10/23/2019
@author: Robert Penicka
'''
class ServiceSetBrickPlacedIntoPlanKeeper(EventState):
    '''
    State for saving the plans for robots in Wall challenge in MBZIRC 2020 into plan_keeper node.
        -- service_topic 	string      Service_topic_name.

        ># brick_id                 Plans how to divide robots in challenge.

        <= successed                Whenever the calling for successful.
        <= failed 		            When the calling failed.

    '''

    def __init__(self, service_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceSetBrickPlacedIntoPlanKeeper, self).__init__(input_keys=['brick_id','placing_result'],
                                                           outcomes = ['successed', 'failed'])

        # Store state parameter for later use.
        self._service_topic = service_topic

        # Create proxy service client
        self._srv = ProxyServiceCaller({self._service_topic: plan_keeper.srv.BrickResult})

        self._failed = False

        Logger.loginfo("[ServiceSetBrickPlacedIntoPlanKeeper]: init with service_topic %s"%(str(self._service_topic)))


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
        Logger.loginfo("[ServiceSetBrickPlacedIntoPlanKeeper]: entering state")
        # Check if the ProxyServiceCaller has been registered
        if not self._srv.is_available(self._service_topic):
            Logger.logerr('[ServiceSetBrickPlacedIntoPlanKeeper]: Topic \'{}\' not yet registered!'.format(self._service_topic))
            self._failed = True
            return

        try:

            service_request = plan_keeper.srv.BrickResultRequest()
            service_request.brick_id = userdata.brick_id
            service_request.result = userdata.placing_result

            Logger.loginfo("[ServiceSetBrickPlacedIntoPlanKeeper]: calling service %s"%(str(self._service_topic)))
            service_result = self._srv.call(self._service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('[ServiceSetBrickPlacedIntoPlanKeeper]: Calling \'{}\' was not successful: {}'.format(self._service_topic, str(service_result.message)))
            else:
                Logger.loginfo('[ServiceSetBrickPlacedIntoPlanKeeper]: service result is %s'%(str(service_result.message)))

        except Exception as e:
            Logger.logerr('[ServiceSetBrickPlacedIntoPlanKeeper]: Failed to call \'{}\' service request: {}'.format(self._service_topic, str(e)))
            self._failed = True
