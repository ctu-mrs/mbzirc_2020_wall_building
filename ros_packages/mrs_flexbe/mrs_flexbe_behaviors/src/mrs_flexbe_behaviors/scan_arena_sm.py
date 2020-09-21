#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mrs_flexbe_states.service_set_brick_detection_type_and_layer import ServiceSetBrickDetectionTypeAndWallLayer
from mrs_flexbe_states.service_follow_trajectory import ServiceFollowTrajectory
from mrs_flexbe_states.service_goto_reference_state import ServiceGoToReferenceState
from mrs_flexbe_states.service_reset_map_state import ServiceResetMapState
from mrs_flexbe_states.service_get_wall_brick_positions import ServiceGetWallBrickPositionsState
from mrs_flexbe_states.service_scan_arena import ServiceScanArena
from mrs_flexbe_states.service_setbool_state import ServiceSetBoolState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import rospy
# [/MANUAL_IMPORT]


'''
Created on Fri Jan 31 2020
@author: Robert Penicka
'''
class scan_arenaSM(Behavior):
  '''
  scanning mbzirc arena
1) get scanning trajectory from arena_scan_planner
2) fly to start
3) follow trajectory
  '''


  def __init__(self):
    super(scan_arenaSM, self).__init__()
    self.name = 'scan_arena'

    # parameters of this behavior

    # references to used behaviors

    # Additional initialization code can be added inside the following tags
    # [MANUAL_INIT]
    
    # [/MANUAL_INIT]

    # Behavior comments:



  def create(self):
    # x:1206 y:204, x:32 y:366
    _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['scanning_speed', 'scanning_robots', 'scanning_altitude', 'scanning_max_acc', 'scanning_turning_speed', 'robot_id'], output_keys=['mapped_objects'])
    _state_machine.userdata.scanning_altitude = 8.0
    _state_machine.userdata.scanning_speed = 5.0
    _state_machine.userdata.scanning_robots = 1
    _state_machine.userdata.scanning_max_acc = 2.0
    _state_machine.userdata.scanning_turning_speed = 2.0
    _state_machine.userdata.robot_id = 0
    _state_machine.userdata.frame_id = "gps_origin"
    _state_machine.userdata.mapped_objects = []
    _state_machine.userdata.BRICK_SEE_EVERYTHING = 0
    _state_machine.userdata.LAYER_0 = 0

    # Additional creation code can be added inside the following tags
    # [MANUAL_CREATE]

    param_prefix_namespace = "flexbe_behavior_launcher/"
    robot_name = rospy.get_param(param_prefix_namespace + 'robot_name')
    frame_id = rospy.get_param(param_prefix_namespace + 'main/frame_id')

    flyto_server_name = rospy.get_param(param_prefix_namespace + 'main/flyto_server_name')
    flyto_server_name = "/" + robot_name + "/" + flyto_server_name
    Logger.loginfo("flyto_server_name: %s" % flyto_server_name)

    scan_planner_service_topic = rospy.get_param(param_prefix_namespace + 'arena_scanning/scan_planner_service_topic')
    scan_planner_service_topic = "/" + robot_name + "/" + scan_planner_service_topic
    Logger.loginfo("scan_planner_service_topic: %s" % scan_planner_service_topic)
    
    reset_map_service_topic = rospy.get_param(param_prefix_namespace + 'arena_scanning/reset_map_service_topic')
    reset_map_service_topic = "/" + robot_name + "/" + reset_map_service_topic
    Logger.loginfo("reset_map_service_topic: %s" % reset_map_service_topic)
    
    get_wall_brick_positions_service_topic = rospy.get_param(param_prefix_namespace + 'arena_scanning/get_wall_brick_positions_service_topic')
    get_wall_brick_positions_service_topic = "/" + robot_name + "/" + get_wall_brick_positions_service_topic
    Logger.loginfo("get_wall_brick_positions_service_topic: %s" % get_wall_brick_positions_service_topic)
    
    start_trajectory_following_service_topic = rospy.get_param(param_prefix_namespace + 'arena_scanning/start_trajectory_following_service_topic')
    start_trajectory_following_service_topic = "/" + robot_name + "/" + start_trajectory_following_service_topic
    Logger.loginfo("start_trajectory_following_service_topic: %s" % start_trajectory_following_service_topic)

    set_trajectory_service_topic = rospy.get_param(param_prefix_namespace + 'arena_scanning/set_trajectory_service_topic')
    set_trajectory_service_topic = "/" + robot_name + "/" + set_trajectory_service_topic
    Logger.loginfo("set_trajectory_service_topic: %s" % set_trajectory_service_topic)
    
    goto_service_topic = rospy.get_param(param_prefix_namespace + 'main/goto_service_topic')
    goto_service_topic = "/" + robot_name + "/" + goto_service_topic
    Logger.loginfo("goto_service_topic: %s" % goto_service_topic)
    
    goto_reference_service_topic = rospy.get_param(param_prefix_namespace + 'main/goto_reference_service_topic')
    goto_reference_service_topic = "/" + robot_name + "/" + goto_reference_service_topic
    Logger.loginfo("goto_reference_service_topic: %s" % goto_reference_service_topic)

    control_manager_diagnostics_topic = rospy.get_param(param_prefix_namespace + 'main/control_manager_diagnostics_topic')
    control_manager_diagnostics_topic = "/" + robot_name + "/" + control_manager_diagnostics_topic
    Logger.loginfo("control_manager_diagnostics_topic: %s" % control_manager_diagnostics_topic)

    brick_detection_type_service_topic = rospy.get_param(param_prefix_namespace + 'main/brick_detection_type_service_topic')
    brick_detection_type_service_topic = "/" + robot_name + "/" + brick_detection_type_service_topic
    Logger.loginfo("brick_detection_type_service_topic: %s" % brick_detection_type_service_topic)
    
    wall_layer_service_topic = rospy.get_param(param_prefix_namespace + 'main/wall_layer_service_topic')
    wall_layer_service_topic = "/" + robot_name + "/" + wall_layer_service_topic
    Logger.loginfo("wall_layer_service_topic: %s" % wall_layer_service_topic)

    brick_estimation_stop_map = rospy.get_param(param_prefix_namespace + 'main/brick_estimation_stop_map')
    brick_estimation_stop_map = "/" + robot_name + "/" + brick_estimation_stop_map
    Logger.loginfo("brick_estimation_stop_map: %s" % brick_estimation_stop_map)
    #arena scanning params  part end
    # [/MANUAL_CREATE]


    with _state_machine:
      # x:129 y:35
      OperatableStateMachine.add('See_All',
                    ServiceSetBrickDetectionTypeAndWallLayer(service_topic_see_type=brick_detection_type_service_topic, service_topic_layer=wall_layer_service_topic),
                    transitions={'successed': 'GetScanPlan', 'failed': 'GetScanPlan'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'see_type': 'BRICK_SEE_EVERYTHING', 'wall_layer': 'LAYER_0'})

      # x:686 y:280
      OperatableStateMachine.add('FollowTrajectory',
                    ServiceFollowTrajectory(service_topic_follow=start_trajectory_following_service_topic, service_topic_set_trajectory=set_trajectory_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'StopMapPublishing', 'failed': 'failed'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'scanning_trajectory': 'scanning_trajectory', 'frame_id': 'frame_id'})

      # x:764 y:61
      OperatableStateMachine.add('FlyToTrajectoryStart',
                    ServiceGoToReferenceState(service_topic=goto_reference_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'StartMapPublishing', 'failed': 'FlyToTrajectoryStart'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal_tracker_point': 'scanning_trajectory_start', 'frame_id': 'frame_id'})

      # x:452 y:39
      OperatableStateMachine.add('ResetMap',
                    ServiceResetMapState(service_topic=reset_map_service_topic),
                    transitions={'finished': 'FlyToTrajectoryStart', 'failed': 'failed'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:907 y:388
      OperatableStateMachine.add('GetMappedPositions',
                    ServiceGetWallBrickPositionsState(service_topic=get_wall_brick_positions_service_topic),
                    transitions={'finished': 'finished', 'failed': 'failed'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'mapped_objects': 'mapped_objects'})

      # x:61 y:169
      OperatableStateMachine.add('GetScanPlan',
                    ServiceScanArena(service_topic=scan_planner_service_topic),
                    transitions={'successed': 'ResetMap', 'failed': 'failed'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'scanning_robots': 'scanning_robots', 'scanning_altitude': 'scanning_altitude', 'scanning_speed': 'scanning_speed', 'scanning_max_acc': 'scanning_max_acc', 'scanning_turning_speed': 'scanning_turning_speed', 'robot_id': 'robot_id', 'scanning_trajectory': 'scanning_trajectory', 'scanning_trajectory_start': 'scanning_trajectory_start'})

      # x:951 y:236
      OperatableStateMachine.add('StopMapPublishing',
                    ServiceSetBoolState(service_topic=brick_estimation_stop_map, request=False, state_name=None),
                    transitions={'finished': 'GetMappedPositions', 'failed': 'GetMappedPositions'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:763 y:172
      OperatableStateMachine.add('StartMapPublishing',
                    ServiceSetBoolState(service_topic=brick_estimation_stop_map, request=True, state_name=None),
                    transitions={'finished': 'FollowTrajectory', 'failed': 'FollowTrajectory'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})


    return _state_machine


  # Private functions can be added inside the following tags
  # [MANUAL_FUNC]
  
  # [/MANUAL_FUNC]
