#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from mrs_flexbe_states.lidar_flyer_action_state import LidarFlierActionState
from mrs_flexbe_states.wait_for_msg_state import WaitForMsgState
from mrs_flexbe_states.fire_extinguish_action_state import FireExtinguishActionState
from mrs_flexbe_states.service_string_state import ServiceStringState
from flexbe_states.check_condition_state import CheckConditionState
from mrs_flexbe_states.set_variable_state import SetVariableToTrueState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import math
import rospy
import mrs_msgs.msg
import mrs_msgs.srv
import lidar_flier.msg
import fire_detect.msg
import copy
import geometry_msgs.msg
# [/MANUAL_IMPORT]


'''
Created on Tue Sep 17 2019
@author: Vojtech Spurny
'''
class fire_challenge_outsideSM(Behavior):
  '''
  Main state machine for the third challenge of MBZIRC 2020
  '''


  def __init__(self):
    super(fire_challenge_outsideSM, self).__init__()
    self.name = 'fire_challenge_outside'

    # parameters of this behavior

    # references to used behaviors

    # Additional initialization code can be added inside the following tags
    # [MANUAL_INIT]
    
    # [/MANUAL_INIT]

    # Behavior comments:



  def create(self):
    # x:214 y:238, x:1435 y:649, x:153 y:589, x:383 y:502, x:1263 y:790
    _state_machine = OperatableStateMachine(outcomes=['no_detection', 'water_depleted', 'mission_part_failure', 'preempted', 'target_lost'])
    _state_machine.userdata.flying_altitude = 5
    _state_machine.userdata.orbit_goal = lidar_flier.msg.lfGoal()
    _state_machine.userdata.fire_position = fire_detect.msg.firemanGoal()
    _state_machine.userdata.fire_extinguish_position = lidar_flier.msg.lfGoal()
    _state_machine.userdata.is_preempted = False
    _state_machine.userdata.is_water_depleted = False
    _state_machine.userdata.is_target_lost = False
    _state_machine.userdata.lidar_flier_speed_goto = 0.5

    # Additional creation code can be added inside the following tags
    # [MANUAL_CREATE]
    
    param_prefix_namespace = "flexbe_behavior_launcher/";
    flying_altitude_list = rospy.get_param(param_prefix_namespace + 'main/flying_altitudes')
    uav_name_list = rospy.get_param(param_prefix_namespace + 'main/robot_name_list')
    robot_name = rospy.get_param(param_prefix_namespace + 'robot_name')
    lidar_flier_speed_goto = rospy.get_param(param_prefix_namespace + 'lidar_flier/lidar_flier_speed_goto')
    lidar_flier_speed_orbit = rospy.get_param(param_prefix_namespace + 'lidar_flier/lidar_flier_speed_orbit')
    lidar_flier_clockwise = rospy.get_param(param_prefix_namespace + 'lidar_flier/lidar_flier_clockwise')
    lidar_flier_stick_distance = rospy.get_param(param_prefix_namespace + 'lidar_flier/lidar_flier_stick_distance')

    fire_detection_topic = rospy.get_param(param_prefix_namespace + 'main/fire_detection_topic')
    fire_detection_topic = "/" + robot_name + "/" + fire_detection_topic
    Logger.loginfo("fire_detection_topic: %s" % fire_detection_topic)

    lidar_flier_action_server_name = rospy.get_param(param_prefix_namespace + 'main/lidar_flier_action_server_name')
    lidar_flier_action_server_name = "/" + robot_name + "/" + lidar_flier_action_server_name
    Logger.loginfo("lidar_flier_action_server_name: %s" % lidar_flier_action_server_name)

    fire_extinguish_action_server_name = rospy.get_param(param_prefix_namespace + 'main/fire_extinguish_action_server_name')
    fire_extinguish_action_server_name = "/" + robot_name + "/" + fire_extinguish_action_server_name
    Logger.loginfo("fire_extinguish_action_server_name: %s" % fire_extinguish_action_server_name)

    control_manager_diagnostics_topic = rospy.get_param(param_prefix_namespace + 'main/control_manager_diagnostics_topic')
    control_manager_diagnostics_topic = "/" + robot_name + "/" + control_manager_diagnostics_topic
    Logger.loginfo("control_manager_diagnostics_topic: %s" % control_manager_diagnostics_topic)

    control_manager_cmd_topic = rospy.get_param(param_prefix_namespace + 'main/control_manager_cmd_topic')
    control_manager_cmd_topic = "/" + robot_name + "/" + control_manager_cmd_topic
    Logger.loginfo("control_manager_cmd_topic: %s" % control_manager_cmd_topic)

    constraint_switch_service_topic = rospy.get_param(param_prefix_namespace + 'main/constraint_switch_service_topic')
    constraint_switch_service_topic = "/" + robot_name + "/" + constraint_switch_service_topic
    Logger.loginfo("constraint_switch_service_topic: %s" % constraint_switch_service_topic)

    control_manager_switch_controller_service_topic = rospy.get_param(param_prefix_namespace + 'main/control_manager_switch_controller_service_topic')
    control_manager_switch_controller_service_topic = "/" + robot_name + "/" + control_manager_switch_controller_service_topic
    Logger.loginfo("control_manager_switch_controller_service_topic: %s" % control_manager_switch_controller_service_topic)

    outside_flying_controller = rospy.get_param(param_prefix_namespace + 'main/outside_flying_controller')
    outside_flying_controller = outside_flying_controller
    Logger.loginfo("outside_flying_controller: %s" % outside_flying_controller)

    extinguishing_controller = rospy.get_param(param_prefix_namespace + 'main/extinguishing_controller')
    extinguishing_controller = extinguishing_controller
    Logger.loginfo("extinguishing_controller: %s" % extinguishing_controller)

    outside_flying_constraints = rospy.get_param(param_prefix_namespace + 'main/outside_flying_constraints')
    outside_flying_constraints = outside_flying_constraints
    Logger.loginfo("outside_flying_constraints: %s" % outside_flying_constraints)

    extinguishing_constraints = rospy.get_param(param_prefix_namespace + 'main/extinguishing_constraints')
    extinguishing_constraints = extinguishing_constraints
    Logger.loginfo("extinguishing_constraints: %s" % extinguishing_constraints)

    flying_altitude = 3

    for i in range(len(uav_name_list)):
      if uav_name_list[i] == robot_name:
        flying_altitude = flying_altitude_list[i]
        break

    orbit_goal = lidar_flier.msg.lfGoal()
    orbit_goal.altitude = flying_altitude
    orbit_goal.orbit = True
    orbit_goal.frame_id = str(robot_name + "/gps_origin")
    orbit_goal.speed = lidar_flier_speed_orbit
    orbit_goal.stick_distance = lidar_flier_stick_distance
    orbit_goal.clockwise = lidar_flier_clockwise
    
    _state_machine.userdata.flying_altitude = flying_altitude 
    _state_machine.userdata.orbit_goal = orbit_goal 
    _state_machine.userdata.lidar_flier_speed_goto = lidar_flier_speed_goto

    Logger.loginfo("fire_ch_sm.flying_altitude: %s" % _state_machine.userdata.flying_altitude)
    Logger.loginfo("fire_ch_sm.orbit_goal: %s" % _state_machine.userdata.orbit_goal)
    Logger.loginfo("fire_ch_sm.lidar_flier_speed_goto: %s" % _state_machine.userdata.lidar_flier_speed_goto)
    
    # [/MANUAL_CREATE]

    # x:30 y:490, x:394 y:528, x:276 y:624, x:711 y:486, x:30 y:576, x:530 y:490, x:630 y:490, x:894 y:493, x:614 y:408, x:778 y:489, x:591 y:614, x:687 y:612, x:1230 y:490
    _sm_lookforfire_0 = ConcurrencyContainer(outcomes=['error', 'preempted', 'no_detection', 'fire_detected'], input_keys=['orbit_goal', 'fire_position', 'lidar_flier_speed_goto', 'fire_extinguish_position'], output_keys=['fire_position', 'fire_extinguish_position'], conditions=[
                    ('error', [('LidarFlierOrbit', 'error')]),
                    ('preempted', [('LidarFlierOrbit', 'preempted')]),
                    ('error', [('LidarFlierOrbit', 'server_not_initialized')]),
                    ('error', [('LidarFlierOrbit', 'stop_requested')]),
                    ('error', [('LidarFlierOrbit', 'no_valid_points_in_scan')]),
                    ('no_detection', [('LidarFlierOrbit', 'successed')]),
                    ('fire_detected', [('WaitForDetection', 'successed')]),
                    ('error', [('WaitForDetection', 'failed')]),
                    ('error', [('InfiniteWait', 'done')])
                    ])

    with _sm_lookforfire_0:
      # x:537 y:175
      OperatableStateMachine.add('InfiniteWait',
                    WaitState(wait_time=1000000),
                    transitions={'done': 'error'},
                    autonomy={'done': Autonomy.Off})

      # x:118 y:146
      OperatableStateMachine.add('LidarFlierOrbit',
                    LidarFlierActionState(action_server_name=lidar_flier_action_server_name),
                    transitions={'successed': 'no_detection', 'no_valid_points_in_scan': 'error', 'stop_requested': 'error', 'server_not_initialized': 'error', 'error': 'error', 'preempted': 'preempted'},
                    autonomy={'successed': Autonomy.Off, 'no_valid_points_in_scan': Autonomy.Off, 'stop_requested': Autonomy.Off, 'server_not_initialized': Autonomy.Off, 'error': Autonomy.Off, 'preempted': Autonomy.Off},
                    remapping={'goal': 'orbit_goal'})

      # x:828 y:141
      OperatableStateMachine.add('WaitForDetection',
                    WaitForMsgState(topic=fire_detection_topic, wait_time=-1, function=self.callback_true, input_keys=['lidar_flier_speed_goto'], output_keys=['fire_extinguish_position','fire_position'], output_function=self.fire_detect_output_function),
                    transitions={'successed': 'fire_detected', 'failed': 'error'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'lidar_flier_speed_goto': 'lidar_flier_speed_goto', 'fire_extinguish_position': 'fire_extinguish_position', 'fire_position': 'fire_position'})



    with _state_machine:
      # x:322 y:26
      OperatableStateMachine.add('LookForFire',
                    _sm_lookforfire_0,
                    transitions={'error': 'mission_part_failure', 'preempted': 'preempted', 'no_detection': 'no_detection', 'fire_detected': 'LidarFlierGotoExtinguishPosition'},
                    autonomy={'error': Autonomy.Inherit, 'preempted': Autonomy.Inherit, 'no_detection': Autonomy.Inherit, 'fire_detected': Autonomy.Inherit},
                    remapping={'orbit_goal': 'orbit_goal', 'fire_position': 'fire_position', 'lidar_flier_speed_goto': 'lidar_flier_speed_goto', 'fire_extinguish_position': 'fire_extinguish_position'})

      # x:856 y:209
      OperatableStateMachine.add('FireExtinguish',
                    FireExtinguishActionState(action_server_name=fire_extinguish_action_server_name),
                    transitions={'successed': 'SetExtinguishingAsSuccessful', 'lost_target': 'SetTargetAsLost', 'error': 'SwitchControlletForFlyingOutside', 'preempted': 'SetPreempted'},
                    autonomy={'successed': Autonomy.Off, 'lost_target': Autonomy.Off, 'error': Autonomy.Off, 'preempted': Autonomy.Off},
                    remapping={'goal': 'fire_position', 'is_tank_depleted': 'is_tank_depleted'})

      # x:543 y:27
      OperatableStateMachine.add('LidarFlierGotoExtinguishPosition',
                    LidarFlierActionState(action_server_name=lidar_flier_action_server_name),
                    transitions={'successed': 'SwitchControllerForExtinguishing', 'no_valid_points_in_scan': 'mission_part_failure', 'stop_requested': 'mission_part_failure', 'server_not_initialized': 'mission_part_failure', 'error': 'mission_part_failure', 'preempted': 'preempted'},
                    autonomy={'successed': Autonomy.Off, 'no_valid_points_in_scan': Autonomy.Off, 'stop_requested': Autonomy.Off, 'server_not_initialized': Autonomy.Off, 'error': Autonomy.Off, 'preempted': Autonomy.Off},
                    remapping={'goal': 'fire_extinguish_position'})

      # x:848 y:29
      OperatableStateMachine.add('SwitchControllerForExtinguishing',
                    ServiceStringState(service_topic=control_manager_switch_controller_service_topic, msg_text=extinguishing_controller, state_name="ControlManager"),
                    transitions={'finished': 'SwitchConstraintsForExtinguishing', 'failed': 'SwitchControllerForExtinguishing'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:838 y:416
      OperatableStateMachine.add('SwitchControlletForFlyingOutside',
                    ServiceStringState(service_topic=control_manager_switch_controller_service_topic, msg_text=outside_flying_controller, state_name="ControlManager"),
                    transitions={'finished': 'SwitchConstraintsForOutsideFlying', 'failed': 'SwitchControlletForFlyingOutside'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:837 y:128
      OperatableStateMachine.add('SwitchConstraintsForExtinguishing',
                    ServiceStringState(service_topic=constraint_switch_service_topic, msg_text=extinguishing_constraints, state_name="ControlManager"),
                    transitions={'finished': 'FireExtinguish', 'failed': 'SwitchConstraintsForExtinguishing'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:832 y:520
      OperatableStateMachine.add('SwitchConstraintsForOutsideFlying',
                    ServiceStringState(service_topic=constraint_switch_service_topic, msg_text=outside_flying_constraints, state_name="ControlManager"),
                    transitions={'finished': 'WasExtinguishingSuccessful', 'failed': 'SwitchConstraintsForOutsideFlying'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:873 y:717
      OperatableStateMachine.add('WasPreempted',
                    CheckConditionState(predicate=lambda x: x == True),
                    transitions={'true': 'preempted', 'false': 'WasTargetLost'},
                    autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                    remapping={'input_value': 'is_preempted'})

      # x:854 y:622
      OperatableStateMachine.add('WasExtinguishingSuccessful',
                    CheckConditionState(predicate=lambda x: x == True),
                    transitions={'true': 'water_depleted', 'false': 'WasPreempted'},
                    autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                    remapping={'input_value': 'is_water_depleted'})

      # x:873 y:802
      OperatableStateMachine.add('WasTargetLost',
                    CheckConditionState(predicate=lambda x: x==True),
                    transitions={'true': 'target_lost', 'false': 'mission_part_failure'},
                    autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                    remapping={'input_value': 'is_target_lost'})

      # x:1253 y:309
      OperatableStateMachine.add('SetPreempted',
                    SetVariableToTrueState(key=['is_preempted'], function=self.set_preempted_variable),
                    transitions={'done': 'SwitchControlletForFlyingOutside'},
                    autonomy={'done': Autonomy.Off},
                    remapping={'is_preempted': 'is_preempted'})

      # x:675 y:310
      OperatableStateMachine.add('SetExtinguishingAsSuccessful',
                    SetVariableToTrueState(key=['is_water_depleted'], function=self.set_water_depleted_variable),
                    transitions={'done': 'SwitchControlletForFlyingOutside'},
                    autonomy={'done': Autonomy.Off},
                    remapping={'is_water_depleted': 'is_water_depleted'})

      # x:979 y:310
      OperatableStateMachine.add('SetTargetAsLost',
                    SetVariableToTrueState(key=['is_target_lost'], function=self.set_target_lost_variable),
                    transitions={'done': 'SwitchControlletForFlyingOutside'},
                    autonomy={'done': Autonomy.Off},
                    remapping={'is_target_lost': 'is_target_lost'})


    return _state_machine


  # Private functions can be added inside the following tags
  # [MANUAL_FUNC]
  @staticmethod
  def callback_true(message):
      return True
  
  @staticmethod
  def fire_detect_output_function(message, userdata):
      fire_extinguish_position = lidar_flier.msg.lfGoal()
      fire_extinguish_position.altitude = message.reference.position.z
      fire_extinguish_position.goto_x = message.reference.position.x
      fire_extinguish_position.goto_y = message.reference.position.y
      fire_extinguish_position.goto_yaw = message.reference.yaw
      fire_extinguish_position.orbit = False
      fire_extinguish_position.frame_id = message.header.frame_id
      fire_extinguish_position.speed = userdata.lidar_flier_speed_goto
      userdata.fire_extinguish_position = copy.deepcopy(fire_extinguish_position)

      point = geometry_msgs.msg.Point()
      point.x = message.reference.position.x
      point.y = message.reference.position.y
      point.z = message.reference.position.z
      userdata.fire_position = copy.deepcopy(point)

  @staticmethod
  def set_water_depleted_variable(userdata):
      userdata.is_water_depleted = True

  @staticmethod
  def set_preempted_variable(userdata):
      userdata.is_preempted = True
  @staticmethod
  def set_target_lost_variable(userdata):
      userdata.is_target_lost = True

  # [/MANUAL_FUNC]
