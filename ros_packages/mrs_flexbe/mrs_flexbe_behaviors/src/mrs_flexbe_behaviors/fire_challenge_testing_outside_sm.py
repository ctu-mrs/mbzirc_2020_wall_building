#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mrs_flexbe_states.wait_for_start_service_call import ServiceWaitForStart
from mrs_flexbe_states.service_goto_altitude_state import ServiceGoToAltitudeState
from mrs_flexbe_states.wait_for_msg_state import WaitForMsgState
from mrs_flexbe_states.lidar_flyer_action_state import LidarFlierActionState
from mrs_flexbe_states.service_trigger_state import ServiceTriggerState
from mrs_flexbe_behaviors.fire_challenge_outside_sm import fire_challenge_outsideSM
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
class fire_challenge_testing_outsideSM(Behavior):
  '''
  Main state machine for the third challenge of MBZIRC 2020
  '''


  def __init__(self):
    super(fire_challenge_testing_outsideSM, self).__init__()
    self.name = 'fire_challenge_testing_outside'

    # parameters of this behavior

    # references to used behaviors
    self.add_behavior(fire_challenge_outsideSM, 'fire_challenge_outside')

    # Additional initialization code can be added inside the following tags
    # [MANUAL_INIT]
    
    # [/MANUAL_INIT]

    # Behavior comments:



  def create(self):
    # x:1333 y:90, x:1333 y:340
    _state_machine = OperatableStateMachine(outcomes=['mission_successful', 'missing_failure'])
    _state_machine.userdata.fire_extinguish_position = lidar_flier.msg.lfGoal()
    _state_machine.userdata.lidar_flier_speed_goto = 1
    _state_machine.userdata.fire_position = fire_detect.msg.firemanGoal()
    _state_machine.userdata.orbit_goal = lidar_flier.msg.lfGoal()
    _state_machine.userdata.flying_altitude = 5
    _state_machine.userdata.is_mission_successful = False

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

    window_position_estimation_reset_topic = rospy.get_param(param_prefix_namespace + 'main/window_position_estimation_reset_topic')
    window_position_estimation_reset_topic = "/" + robot_name + "/" + window_position_estimation_reset_topic
    Logger.loginfo("window_position_estimation_reset_topic: %s" % window_position_estimation_reset_topic)

    window_position_estimation_closest_window_topic = rospy.get_param(param_prefix_namespace + 'main/window_position_estimation_closest_window_topic')
    window_position_estimation_closest_window_topic = "/" + robot_name + "/" + window_position_estimation_closest_window_topic
    Logger.loginfo("window_position_estimation_closest_window_topic: %s" % window_position_estimation_closest_window_topic)

    land_service_topic = rospy.get_param(param_prefix_namespace + 'main/land_service_topic')
    land_service_topic = "/" + robot_name + "/" + land_service_topic
    Logger.loginfo("land_service_topic: %s" % land_service_topic)

    e_land_service_topic = rospy.get_param(param_prefix_namespace + 'main/e_land_service_topic')
    e_land_service_topic = "/" + robot_name + "/" + e_land_service_topic
    Logger.loginfo("e_land_service_topic: %s" % e_land_service_topic)

    fire_detection_topic = rospy.get_param(param_prefix_namespace + 'main/fire_detection_topic')
    fire_detection_topic = "/" + robot_name + "/" + fire_detection_topic
    Logger.loginfo("fire_detection_topic: %s" % fire_detection_topic)

    window_flyer_action_server_name = rospy.get_param(param_prefix_namespace + 'main/window_flyer_action_server_name')
    window_flyer_action_server_name = "/" + robot_name + "/" + window_flyer_action_server_name
    Logger.loginfo("window_flyer_action_server_name: %s" % window_flyer_action_server_name)

    lidar_flier_action_server_name = rospy.get_param(param_prefix_namespace + 'main/lidar_flier_action_server_name')
    lidar_flier_action_server_name = "/" + robot_name + "/" + lidar_flier_action_server_name
    Logger.loginfo("lidar_flier_action_server_name: %s" % lidar_flier_action_server_name)

    fire_extinguish_action_server_name = rospy.get_param(param_prefix_namespace + 'main/fire_extinguish_action_server_name')
    fire_extinguish_action_server_name = "/" + robot_name + "/" + fire_extinguish_action_server_name
    Logger.loginfo("fire_extinguish_action_server_name: %s" % fire_extinguish_action_server_name)

    control_manager_diagnostics_topic = rospy.get_param(param_prefix_namespace + 'main/control_manager_diagnostics_topic')
    control_manager_diagnostics_topic = "/" + robot_name + "/" + control_manager_diagnostics_topic
    Logger.loginfo("control_manager_diagnostics_topic: %s" % control_manager_diagnostics_topic)

    wait_for_start_service_topic = rospy.get_param(param_prefix_namespace + 'main/wait_for_start_service_topic')
    wait_for_start_service_topic = "/" + robot_name + "/" + wait_for_start_service_topic
    Logger.loginfo("wait_for_start_service_topic: %s" % wait_for_start_service_topic)

    control_manager_cmd_topic = rospy.get_param(param_prefix_namespace + 'main/control_manager_cmd_topic')
    control_manager_cmd_topic = "/" + robot_name + "/" + control_manager_cmd_topic
    Logger.loginfo("control_manager_cmd_topic: %s" % control_manager_cmd_topic)

    goto_altitude_service_topic = rospy.get_param(param_prefix_namespace + 'main/goto_altitude_service_topic')
    goto_altitude_service_topic = "/" + robot_name + "/" + goto_altitude_service_topic
    Logger.loginfo("goto_altitude_service_topic: %s" % goto_altitude_service_topic)

    constraint_switch_service_topic = rospy.get_param(param_prefix_namespace + 'main/constraint_switch_service_topic')
    constraint_switch_service_topic = "/" + robot_name + "/" + constraint_switch_service_topic
    Logger.loginfo("constraint_switch_service_topic: %s" % constraint_switch_service_topic)

    control_manager_switch_controller_service_topic = rospy.get_param(param_prefix_namespace + 'main/control_manager_switch_controller_service_topic')
    control_manager_switch_controller_service_topic = "/" + robot_name + "/" + control_manager_switch_controller_service_topic
    Logger.loginfo("control_manager_switch_controller_service_topic: %s" % control_manager_switch_controller_service_topic)

    outside_flying_controller = rospy.get_param(param_prefix_namespace + 'main/outside_flying_controller')
    outside_flying_controller = outside_flying_controller
    Logger.loginfo("outside_flying_controller: %s" % outside_flying_controller)

    inside_flying_controller = rospy.get_param(param_prefix_namespace + 'main/inside_flying_controller')
    inside_flying_controller = inside_flying_controller
    Logger.loginfo("inside_flying_controller: %s" % inside_flying_controller)

    extinguishing_controller = rospy.get_param(param_prefix_namespace + 'main/extinguishing_controller')
    extinguishing_controller = extinguishing_controller
    Logger.loginfo("extinguishing_controller: %s" % extinguishing_controller)

    outside_flying_constraints = rospy.get_param(param_prefix_namespace + 'main/outside_flying_constraints')
    outside_flying_constraints = outside_flying_constraints
    Logger.loginfo("outside_flying_constraints: %s" % outside_flying_constraints)

    inside_flying_constraints = rospy.get_param(param_prefix_namespace + 'main/inside_flying_constraints')
    inside_flying_constraints = inside_flying_constraints
    Logger.loginfo("inside_flying_constraints: %s" % inside_flying_constraints)

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


    with _state_machine:
      # x:92 y:74
      OperatableStateMachine.add('WaitForStart',
                    ServiceWaitForStart(service_topic=wait_for_start_service_topic),
                    transitions={'received_one': 'GotoAltitude', 'received_two': 'Land', 'received_three': 'Land', 'error': 'Land'},
                    autonomy={'received_one': Autonomy.Off, 'received_two': Autonomy.Off, 'received_three': Autonomy.Off, 'error': Autonomy.Off},
                    remapping={'start_value': 'start_value'})

      # x:328 y:74
      OperatableStateMachine.add('GotoAltitude',
                    ServiceGoToAltitudeState(service_topic=goto_altitude_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'SaveHomePosition', 'failed': 'Land'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal': 'flying_altitude'})

      # x:641 y:74
      OperatableStateMachine.add('SaveHomePosition',
                    WaitForMsgState(topic=control_manager_cmd_topic, wait_time=5, function=self.callback_true, input_keys=["lidar_flier_speed_goto"], output_keys=["home_position"], output_function=self.home_position_output_function),
                    transitions={'successed': 'fire_challenge_outside', 'failed': 'Land'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'lidar_flier_speed_goto': 'lidar_flier_speed_goto', 'home_position': 'home_position'})

      # x:669 y:224
      OperatableStateMachine.add('LidarFlierGotoHomePosition',
                    LidarFlierActionState(action_server_name=lidar_flier_action_server_name),
                    transitions={'successed': 'Land', 'no_valid_points_in_scan': 'Land', 'stop_requested': 'Land', 'server_not_initialized': 'Land', 'error': 'Land', 'preempted': 'Land'},
                    autonomy={'successed': Autonomy.Off, 'no_valid_points_in_scan': Autonomy.Off, 'stop_requested': Autonomy.Off, 'server_not_initialized': Autonomy.Off, 'error': Autonomy.Off, 'preempted': Autonomy.Off},
                    remapping={'goal': 'home_position'})

      # x:442 y:374
      OperatableStateMachine.add('Land',
                    ServiceTriggerState(service_topic=land_service_topic, state_name="ControlManager"),
                    transitions={'finished': 'WasMissionSuccessful', 'failed': 'Land'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:869 y:71
      OperatableStateMachine.add('fire_challenge_outside',
                    self.use_behavior(fire_challenge_outsideSM, 'fire_challenge_outside'),
                    transitions={'no_detection': 'LidarFlierGotoHomePosition', 'water_depleted': 'SetMissionSuccess', 'mission_part_failure': 'LidarFlierGotoHomePosition', 'preempted': 'LidarFlierGotoHomePosition', 'target_lost': 'LidarFlierGotoHomePosition'},
                    autonomy={'no_detection': Autonomy.Inherit, 'water_depleted': Autonomy.Inherit, 'mission_part_failure': Autonomy.Inherit, 'preempted': Autonomy.Inherit, 'target_lost': Autonomy.Inherit})

      # x:981 y:374
      OperatableStateMachine.add('WasMissionSuccessful',
                    CheckConditionState(predicate=lambda x: x),
                    transitions={'true': 'mission_successful', 'false': 'missing_failure'},
                    autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                    remapping={'input_value': 'is_mission_successful'})

      # x:983 y:224
      OperatableStateMachine.add('SetMissionSuccess',
                    SetVariableToTrueState(key=['is_mission_successful'], function=self.set_success_variable),
                    transitions={'done': 'LidarFlierGotoHomePosition'},
                    autonomy={'done': Autonomy.Off},
                    remapping={'is_mission_successful': 'is_mission_successful'})


    return _state_machine


  # Private functions can be added inside the following tags
  # [MANUAL_FUNC]
  @staticmethod
  def callback_true(message):
      return True
  
  @staticmethod
  def set_success_variable(userdata):
      userdata.is_mission_successful = True

  @staticmethod
  def home_position_output_function(message, userdata):
      home_position = lidar_flier.msg.lfGoal()
      home_position.altitude = message.pose.pose.position.z
      home_position.goto_x = message.pose.pose.position.x
      home_position.goto_y = message.pose.pose.position.y
      home_position.orbit = False
      home_position.frame_id = message.header.frame_id
      home_position.speed = userdata.lidar_flier_speed_goto
      userdata.home_position = copy.deepcopy(home_position)


  # [/MANUAL_FUNC]
