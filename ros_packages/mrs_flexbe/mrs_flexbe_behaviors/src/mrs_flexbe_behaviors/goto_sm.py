#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mrs_flexbe_states.service_goto_state import ServiceGoToState
from mrs_flexbe_states.wait_for_msg_state import WaitForMsgState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import rospy
# [/MANUAL_IMPORT]


'''
Created on Tue Sep 17 2019
@author: Vojtech Spurny
'''
class GotoSM(Behavior):
    '''
    SM that will call goto service in the MRS system and wait until goal is reached.
    '''


    def __init__(self):
        super(GotoSM, self).__init__()
        self.name = 'Goto'

        # parameters of this behavior

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
	# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        control_goto_topic = "control_manager/goto"
        control_mpc_tracker_diagnostic_topic = "control_manager/mpc_tracker/diagnostics"
        # x:683 y:90, x:283 y:190
        _state_machine = OperatableStateMachine(outcomes=['reached', 'failed'], input_keys=['goal'])
        _state_machine.userdata.goal = [0, 0, 2, 0]

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:97 y:74
            OperatableStateMachine.add('Call_GoTo',
                                        ServiceGoToState(service_topic=control_goto_topic),
                                        transitions={'successed': 'Wait_Till_Reaching_Goal', 'failed': 'failed'},
                                        autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'goal': 'goal'})

            # x:381 y:74
            OperatableStateMachine.add('Wait_Till_Reaching_Goal',
                                        WaitForMsgState(topic=control_mpc_tracker_diagnostic_topic, wait_time=-1, function=self.mpc_tracker_diagnostics_cb),
                                        transitions={'successed': 'reached', 'failed': 'failed'},
                                        autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    @staticmethod
    def mpc_tracker_diagnostics_cb(msg):
        if msg.tracking_trajectory: 
            rospy.loginfo("[GoTo]: Waiting till reaching goal.")
            return False 
        else:
            rospy.loginfo("[GoTo]: Goal reached.")
            return True
	
    # [/MANUAL_FUNC]
