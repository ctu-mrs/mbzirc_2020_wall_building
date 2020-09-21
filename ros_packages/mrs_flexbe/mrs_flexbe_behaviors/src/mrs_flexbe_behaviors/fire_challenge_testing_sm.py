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
from mrs_flexbe_states.service_trigger_state import ServiceTriggerState
from mrs_flexbe_states.service_goto_altitude_state import ServiceGoToAltitudeState
from mrs_flexbe_states.wait_for_msg_state import WaitForMsgState
from mrs_flexbe_states.window_flyer_action_state import WindowFlyerActionState
from flexbe_states.wait_state import WaitState
from mrs_flexbe_states.lidar_flyer_action_state import LidarFlierActionState
from mrs_flexbe_states.service_string_state import ServiceStringState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import math
import rospy
import mrs_msgs.msg
import mrs_msgs.srv
import lidar_flier.msg
import copy
# [/MANUAL_IMPORT]


'''
Created on Tue Sep 17 2019
@author: Vojtech Spurny
'''
class fire_challenge_testingSM(Behavior):
  '''
  Main state machine for the third challenge of MBZIRC 2020
  '''


  def __init__(self):
    super(fire_challenge_testingSM, self).__init__()
    self.name = 'fire_challenge_testing'

    # parameters of this behavior

    # references to used behaviors

    # Additional initialization code can be added inside the following tags
    # [MANUAL_INIT]

    # [/MANUAL_INIT]

    # Behavior comments:



  def create(self):
    # x:192 y:728, x:263 y:340
    _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
    _state_machine.userdata.simulation = True
    _state_machine.userdata.flying_altitude = 5
    _state_machine.userdata.orbit_goal = ""
    _state_machine.userdata.window_id = 0
    _state_machine.userdata.lidar_flier_speed_orbit = 0
    _state_machine.userdata.lidar_flier_speed_goto = 0
    _state_machine.userdata.lidar_flier_clockwise = False
    _state_machine.userdata.lidar_flier_stick_distance = False

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

    control_manager_cmd_topic = rospy.get_param(param_prefix_namespace + 'main/control_manager_cmd_topic')
    control_manager_cmd_topic = "/" + robot_name + "/" + control_manager_cmd_topic
    Logger.loginfo("control_manager_cmd_topic: %s" % control_manager_cmd_topic)

    goto_reference_service_topic = rospy.get_param(param_prefix_namespace + 'main/goto_reference_service_topic')
    goto_reference_service_topic = "/" + robot_name + "/" + goto_reference_service_topic
    Logger.loginfo("goto_reference_service_topic: %s" % goto_reference_service_topic)
    
    goto_altitude_service_topic = rospy.get_param(param_prefix_namespace + 'main/goto_altitude_service_topic')
    goto_altitude_service_topic = "/" + robot_name + "/" + goto_altitude_service_topic
    Logger.loginfo("goto_altitude_service_topic: %s" % goto_altitude_service_topic)

    wait_for_start_service_topic = rospy.get_param(param_prefix_namespace + 'main/wait_for_start_service_topic')
    wait_for_start_service_topic = "/" + robot_name + "/" + wait_for_start_service_topic
    Logger.loginfo("wait_for_start_service_topic: %s" % wait_for_start_service_topic)

    # confirmation_service_topic = rospy.get_param(param_prefix_namespace + 'main/confirmation_service_topic')
    # confirmation_service_topic = "/" + robot_name + "/" + confirmation_service_topic
    # Logger.loginfo("confirmation_service_topic: %s" % confirmation_service_topic)

    hector_reset_service_topic = rospy.get_param(param_prefix_namespace + 'main/hector_reset_service_topic')
    hector_reset_service_topic = "/" + robot_name + "/" + hector_reset_service_topic
    Logger.loginfo("hector_reset_service_topic: %s" % hector_reset_service_topic)

    odometry_switch_service_topic = rospy.get_param(param_prefix_namespace + 'main/odometry_switch_service_topic')
    odometry_switch_service_topic = "/" + robot_name + "/" + odometry_switch_service_topic
    Logger.loginfo("odometry_switch_service_topic: %s" % odometry_switch_service_topic)

    flying_altitude = 3
    for i in range(len(uav_name_list)):
      if uav_name_list[i] == robot_name:
        flying_altitude = flying_altitude_list[i]
        break

    orbit_goal = lidar_flier.msg.lfGoal()
    orbit_goal.altitude = flying_altitude
    orbit_goal.orbit = True
    orbit_goal.speed = lidar_flier_speed_orbit
    orbit_goal.clockwise = lidar_flier_clockwise
    orbit_goal.stick_distance = lidar_flier_stick_distance
    
    _state_machine.userdata.flying_altitude = flying_altitude 
    _state_machine.userdata.orbit_goal = orbit_goal 
    _state_machine.userdata.lidar_flier_speed_goto = lidar_flier_speed_goto
    _state_machine.userdata.lidar_flier_speed_orbit = lidar_flier_speed_orbit
    _state_machine.userdata.lidar_flier_clockwise = lidar_flier_clockwise
    _state_machine.userdata.lidar_flier_stick_distance = lidar_flier_stick_distance

    Logger.loginfo("fire_ch_sm.flying_altitude: %s" % _state_machine.userdata.flying_altitude)
    Logger.loginfo("fire_ch_sm.orbit_goal: %s" % _state_machine.userdata.orbit_goal)
    Logger.loginfo("fire_ch_sm.lidar_flier_speed_goto: %s" % _state_machine.userdata.lidar_flier_speed_goto)
    Logger.loginfo("fire_ch_sm.lidar_flier_speed_orbit: %s" % _state_machine.userdata.lidar_flier_speed_orbit)
    Logger.loginfo("fire_ch_sm.lidar_flier_stick_distance: %s" % _state_machine.userdata.lidar_flier_stick_distance)
    Logger.loginfo("fire_ch_sm.lidar_flier_clockwise: %s" % _state_machine.userdata.lidar_flier_clockwise)
    

    # [/MANUAL_CREATE]

    # x:30 y:490, x:644 y:487, x:230 y:490, x:330 y:490, x:430 y:490, x:35 y:612, x:97 y:605, x:168 y:600, x:1205 y:397, x:883 y:498, x:1030 y:490, x:1130 y:490, x:1230 y:490
    _sm_lookforwindow_0 = ConcurrencyContainer(outcomes=['error', 'detection', 'no_detection', 'preemted'], input_keys=['orbit_goal', 'window_id'], output_keys=['window_id'], conditions=[
                    ('no_detection', [('OrbitBuilding', 'successed')]),
                    ('error', [('OrbitBuilding', 'no_valid_points_in_scan')]),
                    ('error', [('OrbitBuilding', 'stop_requested')]),
                    ('error', [('OrbitBuilding', 'error')]),
                    ('preemted', [('OrbitBuilding', 'preempted')]),
                    ('error', [('OrbitBuilding', 'server_not_initialized')]),
                    ('error', [('InfiniteWait', 'done')]),
                    ('detection', [('WaitForDetection', 'successed')]),
                    ('error', [('WaitForDetection', 'failed')])
                    ])

    with _sm_lookforwindow_0:
      # x:434 y:172
      OperatableStateMachine.add('InfiniteWait',
                    WaitState(wait_time=100000),
                    transitions={'done': 'error'},
                    autonomy={'done': Autonomy.Off})

      # x:155 y:164
      OperatableStateMachine.add('OrbitBuilding',
                    LidarFlierActionState(action_server_name=lidar_flier_action_server_name),
                    transitions={'successed': 'no_detection', 'no_valid_points_in_scan': 'error', 'stop_requested': 'error', 'server_not_initialized': 'error', 'error': 'error', 'preempted': 'preemted'},
                    autonomy={'successed': Autonomy.Off, 'no_valid_points_in_scan': Autonomy.Off, 'stop_requested': Autonomy.Off, 'server_not_initialized': Autonomy.Off, 'error': Autonomy.Off, 'preempted': Autonomy.Off},
                    remapping={'goal': 'orbit_goal'})

      # x:735 y:179
      OperatableStateMachine.add('WaitForDetection',
                    WaitForMsgState(topic=window_position_estimation_closest_window_topic, wait_time=-1, function=self.callback_true, input_keys=["window_id"], output_keys=["window_id"], output_function=self.window_detection_cb),
                    transitions={'successed': 'detection', 'failed': 'error'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'window_id': 'window_id'})



    with _state_machine:
      # x:55 y:69
      OperatableStateMachine.add('WaitForStart',
                    ServiceWaitForStart(service_topic=wait_for_start_service_topic),
                    transitions={'received': 'GotoAltitude'},
                    autonomy={'received': Autonomy.Off},
                    remapping={'start_value': 'start_value'})

      # x:705 y:70
      OperatableStateMachine.add('ResetWindowEstimation',
                    ServiceTriggerState(service_topic=window_position_estimation_reset_topic, state_name="WindowEstimation"),
                    transitions={'finished': 'LookForWindow', 'failed': 'EmergencyLanding'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:519 y:329
      OperatableStateMachine.add('EmergencyLanding',
                    ServiceTriggerState(service_topic=e_land_service_topic, state_name="ControlManager"),
                    transitions={'finished': 'failed', 'failed': 'EmergencyLanding'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:251 y:67
      OperatableStateMachine.add('GotoAltitude',
                    ServiceGoToAltitudeState(service_topic=goto_altitude_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'SaveHomePosition', 'failed': 'EmergencyLanding'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal': 'flying_altitude'})

      # x:495 y:68
      OperatableStateMachine.add('SaveHomePosition',
                    WaitForMsgState(topic=control_manager_cmd_topic, wait_time=5, function=self.callback_true, input_keys=["lidar_flier_speed_goto"], output_keys=["home_position"], output_function=self.home_position_output_function),
                    transitions={'successed': 'ResetWindowEstimation', 'failed': 'EmergencyLanding'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'lidar_flier_speed_goto': 'lidar_flier_speed_goto', 'home_position': 'home_position'})

      # x:1190 y:65
      OperatableStateMachine.add('FlythroughWindowInsideApproach',
                    WindowFlyerActionState(action_server_name=window_flyer_action_server_name, just_approach=True, yaw_offset=3.14),
                    transitions={'successed': 'ResetHector', 'window_not_found': 'EmergencyLanding', 'already_flythrough': 'EmergencyLanding', 'server_not_initialized': 'EmergencyLanding', 'error': 'EmergencyLanding', 'preempted': 'EmergencyLanding'},
                    autonomy={'successed': Autonomy.Off, 'window_not_found': Autonomy.Off, 'already_flythrough': Autonomy.Off, 'server_not_initialized': Autonomy.Off, 'error': Autonomy.Off, 'preempted': Autonomy.Off},
                    remapping={'window_id': 'window_id', 'window_position': 'window_position'})

      # x:1009 y:65
      OperatableStateMachine.add('LookForWindow',
                    _sm_lookforwindow_0,
                    transitions={'error': 'EmergencyLanding', 'detection': 'FlythroughWindowInsideApproach', 'no_detection': 'EmergencyLanding', 'preemted': 'EmergencyLanding'},
                    autonomy={'error': Autonomy.Inherit, 'detection': Autonomy.Inherit, 'no_detection': Autonomy.Inherit, 'preemted': Autonomy.Inherit},
                    remapping={'orbit_goal': 'orbit_goal', 'window_id': 'window_id'})

      # x:1419 y:387
      OperatableStateMachine.add('FlythroughWindowOutsideApproach',
                    WindowFlyerActionState(action_server_name=window_flyer_action_server_name, just_approach=True, yaw_offset=3.14),
                    transitions={'successed': 'FlythroughWindowOutside', 'window_not_found': 'EmergencyLanding', 'already_flythrough': 'EmergencyLanding', 'server_not_initialized': 'EmergencyLanding', 'error': 'EmergencyLanding', 'preempted': 'EmergencyLanding'},
                    autonomy={'successed': Autonomy.Off, 'window_not_found': Autonomy.Off, 'already_flythrough': Autonomy.Off, 'server_not_initialized': Autonomy.Off, 'error': Autonomy.Off, 'preempted': Autonomy.Off},
                    remapping={'window_id': 'window_id', 'window_position': 'window_position'})

      # x:671 y:739
      OperatableStateMachine.add('GoToHome',
                    LidarFlierActionState(action_server_name=lidar_flier_action_server_name),
                    transitions={'successed': 'Land', 'no_valid_points_in_scan': 'EmergencyLanding', 'stop_requested': 'EmergencyLanding', 'server_not_initialized': 'EmergencyLanding', 'error': 'EmergencyLanding', 'preempted': 'EmergencyLanding'},
                    autonomy={'successed': Autonomy.Off, 'no_valid_points_in_scan': Autonomy.Off, 'stop_requested': Autonomy.Off, 'server_not_initialized': Autonomy.Off, 'error': Autonomy.Off, 'preempted': Autonomy.Off},
                    remapping={'goal': 'home_position'})

      # x:390 y:734
      OperatableStateMachine.add('Land',
                    ServiceTriggerState(service_topic=land_service_topic, state_name="ControlManager"),
                    transitions={'finished': 'finished', 'failed': 'Land'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:1467 y:64
      OperatableStateMachine.add('ResetHector',
                    ServiceTriggerState(service_topic=hector_reset_service_topic, state_name="ResetHector"),
                    transitions={'finished': 'WaitForTwoSecond', 'failed': 'EmergencyLanding'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:1627 y:58
      OperatableStateMachine.add('WaitForTwoSecond',
                    WaitState(wait_time=2),
                    transitions={'done': 'SwitchOdometryToHector'},
                    autonomy={'done': Autonomy.Off})

      # x:1444 y:159
      OperatableStateMachine.add('SwitchOdometryToHector',
                    ServiceStringState(service_topic=odometry_switch_service_topic, msg_text="HECTOR", state_name="Odometry"),
                    transitions={'finished': 'WaitForTwoSecond2', 'failed': 'EmergencyLanding'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:1625 y:158
      OperatableStateMachine.add('WaitForTwoSecond2',
                    WaitState(wait_time=2),
                    transitions={'done': 'FlythroughWindowInside'},
                    autonomy={'done': Autonomy.Off})

      # x:999 y:736
      OperatableStateMachine.add('SwitchOdometryToGPS',
                    ServiceStringState(service_topic=odometry_switch_service_topic, msg_text="GPS", state_name="Odometry"),
                    transitions={'finished': 'GoToHome', 'failed': 'EmergencyLanding'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:1455 y:250
      OperatableStateMachine.add('FlythroughWindowInside',
                    WindowFlyerActionState(action_server_name=window_flyer_action_server_name, just_approach=False, yaw_offset=3.14),
                    transitions={'successed': 'FlythroughWindowOutsideApproach', 'window_not_found': 'EmergencyLanding', 'already_flythrough': 'EmergencyLanding', 'server_not_initialized': 'EmergencyLanding', 'error': 'EmergencyLanding', 'preempted': 'EmergencyLanding'},
                    autonomy={'successed': Autonomy.Off, 'window_not_found': Autonomy.Off, 'already_flythrough': Autonomy.Off, 'server_not_initialized': Autonomy.Off, 'error': Autonomy.Off, 'preempted': Autonomy.Off},
                    remapping={'window_id': 'window_id', 'window_position': 'window_position'})

      # x:1448 y:487
      OperatableStateMachine.add('FlythroughWindowOutside',
                    WindowFlyerActionState(action_server_name=window_flyer_action_server_name, just_approach=False, yaw_offset=3.14),
                    transitions={'successed': 'SwitchOdometryToGPS', 'window_not_found': 'EmergencyLanding', 'already_flythrough': 'EmergencyLanding', 'server_not_initialized': 'EmergencyLanding', 'error': 'EmergencyLanding', 'preempted': 'EmergencyLanding'},
                    autonomy={'successed': Autonomy.Off, 'window_not_found': Autonomy.Off, 'already_flythrough': Autonomy.Off, 'server_not_initialized': Autonomy.Off, 'error': Autonomy.Off, 'preempted': Autonomy.Off},
                    remapping={'window_id': 'window_id', 'window_position': 'window_position'})


    return _state_machine


  # Private functions can be added inside the following tags
  # [MANUAL_FUNC]
  @staticmethod
  def callback_true(message):
      return True

  @staticmethod
  def window_detection_cb(message, userdata):
      userdata.window_id = message.objects[0].id

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
