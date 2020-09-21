#!/usr/bin/env python
import rospy
import copy
import mrs_msgs.srv


from flexbe_core import EventState, Logger

'''
Created on 07/02/2020
@author: Robert Penicka
'''


class ServiceWaitForStart(EventState):
    '''
    State for waiting to call from service
        -- service_topic 	string      Service_topic_name.
        

        #> start_value       int with start value
        
        #message SetInt.srv
        int64 value
        ---
        bool success
        string message


        <= successed 		            Whenever the calling for successful.
        <= failed 		                When the calling failed.

    '''

    def result_to_outcome(self, argument):
        switcher = {
            0: "received_one",
            1: "received_two",
            2: "received_three",
        }
        return switcher.get(argument, "error")

    def __init__(self, service_topic ):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(ServiceWaitForStart, self).__init__(output_keys=['start_value'],
                                                        outcomes=['received_one','received_two','received_three','error'])

        # Store state parameter for later use.
        self._service_topic = rospy.resolve_name(service_topic)
        

        # Create proxy service client
        self._srv = rospy.Service(service_topic, mrs_msgs.srv.SetInt, self.service_callback)
        
        self.start_value = 0
        self._finished = False

    def service_callback(self,req):
        Logger.loginfo('[ServiceWaitForStart]: service_callback %s %d'%(self._service_topic,req.value))
        self._start_value = req.value
        self._finished = True
        response = mrs_msgs.srv.SetIntResponse()
        response.success = True
        response.message = "message received with "+str(req.value)
        return response

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.
        
        if self._finished:
            Logger.loginfo('[ServiceWaitForStart]: received - exiting')
            userdata.start_value = self._start_value
            return self.result_to_outcome(self._start_value)
        
        return

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.
        Logger.loginfo('[ServiceWaitForStart]: entering state waiting for callback from %s'%(self._service_topic))
