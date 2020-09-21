#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mrs_flexbe_states.service_setbool_state import ServiceSetBoolState
from mrs_flexbe_states.mavros_arm_state import MavrosArmState
from mrs_flexbe_states.mavros_set_mode_state import MavrosSetModeState
from flexbe_states.check_condition_state import CheckConditionState
from mrs_flexbe_states.service_trigger_state import ServiceTriggerState
from flexbe_states.wait_state import WaitState
from mrs_flexbe_states.wait_for_msg_state import WaitForMsgState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import rospy
# [/MANUAL_IMPORT]


'''
Created on Tue Aug 20 2019
@author: Vojtech Spurny
'''
class Prepare_UAVSM(Behavior):
  '''
  SM that will prepare UAV by sequence: 
- setting motors on 
- arming the FCU (if simulation parameterer is set to True)
- switching the FCU to offboard mode
- takeoff.
Whenever something bad happens, the UAV will lend immediately.
  '''


  def __init__(self):
    super(Prepare_UAVSM, self).__init__()
    self.name = 'Prepare_UAV'

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
    wait_time_for_takeoff = 10
    control_diagnostic_topic = "control_manager/diagnostics"
    e_land_topic = "control_manager/eland"
    # x:1309 y:290, x:588 y:387
    _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['simulation'])
    _state_machine.userdata.simulation = True

    # Additional creation code can be added inside the following tags
    # [MANUAL_CREATE]
		
        # [/MANUAL_CREATE]


    with _state_machine:
      # x:41 y:224
      OperatableStateMachine.add('Set_Motors_ON',
                    ServiceSetBoolState(service_topic=controller_motor_on_topic, request=True, state_name="ControlManager"),
                    transitions={'finished': 'Wait_After_Motors_ON', 'failed': 'failed'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:397 y:174
      OperatableStateMachine.add('Arming',
                    MavrosArmState(request=True),
                    transitions={'finished': 'Set_Offboard', 'failed': 'failed'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:539 y:24
      OperatableStateMachine.add('Set_Offboard',
                    MavrosSetModeState(request="offboard"),
                    transitions={'finished': 'Wait_After_Switching_To_Offboard', 'failed': 'failed'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:240 y:24
      OperatableStateMachine.add('Check_If_To_Arm',
                    CheckConditionState(predicate=lambda simulation: simulation == True),
                    transitions={'true': 'Arming', 'false': 'Set_Offboard'},
                    autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                    remapping={'input_value': 'simulation'})

      # x:793 y:174
      OperatableStateMachine.add('TakeOff',
                    ServiceTriggerState(service_topic=takeoff_topic, state_name="ControlManager"),
                    transitions={'finished': 'Wait_For_Successful_TakeOff', 'failed': 'Emergency_Landing'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:133 y:124
      OperatableStateMachine.add('Wait_After_Motors_ON',
                    WaitState(wait_time=wait_time_after_setting),
                    transitions={'done': 'Check_If_To_Arm'},
                    autonomy={'done': Autonomy.Off})

      # x:754 y:24
      OperatableStateMachine.add('Wait_After_Switching_To_Offboard',
                    WaitState(wait_time=wait_time_after_setting),
                    transitions={'done': 'TakeOff'},
                    autonomy={'done': Autonomy.Off})

      # x:1016 y:274
      OperatableStateMachine.add('Wait_For_Successful_TakeOff',
                    WaitForMsgState(topic=control_diagnostic_topic, wait_time=wait_time_for_takeoff, function=self.mrs_takeoff_cb, output_keys=[], output_function=None),
                    transitions={'successed': 'finished', 'failed': 'Emergency_Landing'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off})

      # x:788 y:324
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
              msg.tracker_status.tracker != "NullTracker" and 
              msg.tracker_status.tracker != "LandoffTracker" and 
              msg.tracker_status.active == True and 
              msg.tracker_status.callbacks_enabled == True
         ):
          rospy.loginfo("[ControlManager]: Takeoff was successfull.")
          return True
      else:
          rospy.loginfo_throttle(1,"[ControlManager]: Waiting for successfull takeoff.")
          return False 
	
    # [/MANUAL_FUNC]
