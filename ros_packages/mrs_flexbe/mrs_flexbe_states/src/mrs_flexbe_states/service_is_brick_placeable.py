    #!/usr/bin/env python
import rospy
import plan_keeper.srv

from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

'''
Created on 10/22/2019
@author: Robert Penicka
'''
class ServiceIsBrickPlaceable(EventState):
    '''
    State for getting whether a brick can be placed in Wall challenge in MBZIRC 2020.
        -- service_topic 	string      Service_topic_name.

        ># brick_id                        Id of current brick being placed

        <= placeable 		        When we can place the brick.
        <= wait                     When we need to wait for placing.
        <= failed 		            When the calling failed.

    '''

    def __init__(self, service_topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceIsBrickPlaceable, self).__init__(input_keys=['brick_id'],
                                                 output_keys=[],
                                                 outcomes = ['placeable', 'wait', 'failed'])

        # Store state parameter for later use.
        self._service_topic = rospy.resolve_name(service_topic)

        # Create proxy service client
        self._srv = ProxyServiceCaller({self._service_topic: plan_keeper.srv.IsBrickPlaceable})

        self._failed = False
        self._placeable = False


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._failed:
            return 'failed'
        else:
            if self._placeable:
                return 'placeable'
            else:
                return 'wait'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        self._failed = False
        self._placeable = False

        # Check if the ProxyServiceCaller has been registered
        if not self._srv.is_available(self._service_topic):
            Logger.logerr('ProxyServiceCaller: Topic \'{}\' not yet registered!'.format(self._service_topic))
            self._failed = True
            return

        try:

            service_request = plan_keeper.srv.IsBrickPlaceableRequest()
            service_request.brick_id = userdata.brick_id
            Logger.loginfo('ask if brick %d is placeable'%(userdata.brick_id))
            service_result = self._srv.call(self._service_topic, service_request)

            if not service_result.success:
                self._failed = True
                Logger.logwarn('Calling \'{}\' was not successful: {}'.format(self._service_topic, str(service_result.message)))
            else:
                Logger.loginfo(service_result.message)
                self._placeable = service_result.placeable

        except Exception as e:
            Logger.logerr('Failed to call \'{}\' service request: {}'.format(self._service_topic, str(e)))
            self._failed = True
