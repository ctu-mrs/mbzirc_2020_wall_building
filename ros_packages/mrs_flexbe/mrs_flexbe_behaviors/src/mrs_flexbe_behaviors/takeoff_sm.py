#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mrs_flexbe_states.service_trigger_state import ServiceTriggerState
from mrs_flexbe_states.wait_for_msg_state import WaitForMsgState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import rospy
# [/MANUAL_IMPORT]


'''
Created on Tue Feb 12 2019
@author: Robert Penicka
'''
class TakeOffSM(Behavior):
  '''
  Only takeoff UAV
  '''


  def __init__(self):
    super(TakeOffSM, self).__init__()
    self.name = 'TakeOff'

    # parameters of this behavior

    # references to used behaviors

    # Additional initialization code can be added inside the following tags
    # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

    # Behavior comments:



  def create(self):
    controller_motor_on_topic = "control_manager/motors"
    takeoff_topic = "uav_manager/takeoff"
    wait_time_after_setting = 0.5
    wait_time_for_takeoff = 15
    control_diagnostic_topic = "control_manager/diagnostics"
    e_land_topic = "control_manager/eland"
    # x:971 y:180, x:588 y:387
    _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['simulation'])
    _state_machine.userdata.simulation = True

    # Additional creation code can be added inside the following tags
    # [MANUAL_CREATE]
		
        # [/MANUAL_CREATE]


    with _state_machine:
      # x:166 y:79
      OperatableStateMachine.add('TakeOff',
                    ServiceTriggerState(service_topic=takeoff_topic, state_name="ControlManager"),
                    transitions={'finished': 'Wait_For_Successful_TakeOff', 'failed': 'Emergency_Landing'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:424 y:142
      OperatableStateMachine.add('Wait_For_Successful_TakeOff',
                    WaitForMsgState(topic=control_diagnostic_topic, wait_time=wait_time_for_takeoff, function=self.mrs_takeoff_cb, input_keys=[], output_keys=[], output_function=None),
                    transitions={'successed': 'finished', 'failed': 'Emergency_Landing'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off})

      # x:196 y:271
      OperatableStateMachine.add('Emergency_Landing',
                    ServiceTriggerState(service_topic=e_land_topic, state_name="ControlManager"),
                    transitions={'finished': 'failed', 'failed': 'failed'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})


    return _state_machine


  # Private functions can be added inside the following tags
  # [MANUAL_FUNC]
  @staticmethod
  def mrs_takeoff_cb(msg):
      if (
              msg.active_tracker != "NullTracker" and 
              msg.active_tracker != "LandoffTracker" and 
              msg.tracker_status.active == True and 
              msg.tracker_status.callbacks_enabled == True
         ):
          rospy.loginfo("[ControlManager]: Takeoff was successfull.")
          return True
      else:
          rospy.loginfo_throttle(1,"[ControlManager]: Waiting for successfull takeoff.")
          return False 
	
    # [/MANUAL_FUNC]
