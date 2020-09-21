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
from mrs_flexbe_states.brick_placing_action_state import BrickPlacingActionState
from mrs_flexbe_states.service_goto_altitude_state import ServiceGoToAltitudeState
from mrs_flexbe_states.service_rotate_relative_to_goto import ServiceRotateRelativeToGoToState
from mrs_flexbe_states.service_goto_reference_state import ServiceGoToReferenceState
from mrs_flexbe_states.service_set_brick_placed_into_plan_keeper import ServiceSetBrickPlacedIntoPlanKeeper
from mrs_flexbe_states.is_gripper_grasped import IsGraspedState
from mrs_flexbe_states.service_goto_reference_with_desired_yaw_state import ServiceGoToReferenceWithDesiredYawState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import rospy
# [/MANUAL_IMPORT]


'''
Created on Wed Feb 12 2020
@author: Robert Penicka
'''
class wall_ch_place_brickSM(Behavior):
  '''
  sm to place brick after grasping
  '''


  def __init__(self):
    super(wall_ch_place_brickSM, self).__init__()
    self.name = 'wall_ch_place_brick'

    # parameters of this behavior

    # references to used behaviors

    # Additional initialization code can be added inside the following tags
    # [MANUAL_INIT]
        # [/MANUAL_INIT]

    # Behavior comments:



  def create(self):
    # x:30 y:463, x:590 y:380, x:619 y:527, x:285 y:383
    _state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'placing_badly', 'dropped_brick'], input_keys=['brick_type', 'placing_position', 'wall_layer', 'flying_altitude', 'fly_with_brick_yaw', 'drop_wait_position', 'brick_id', 'altitude_above_wall', 'brick_see_wall_typed', 'brick_placed_position', 'next_to_wall_wait_position'], output_keys=['desired_yaw', 'brick_placed_position'])
    _state_machine.userdata.brick_type = 1
    _state_machine.userdata.placing_position = [0, 0, 3, 0]
    _state_machine.userdata.BRICK_SEE_EVERYTHING = 0
    _state_machine.userdata.BRICK_SEE_RED = 1
    _state_machine.userdata.BRICK_SEE_GREEN = 2
    _state_machine.userdata.BRICK_SEE_BLUE = 3
    _state_machine.userdata.BRICK_SEE_ORANGE = 4
    _state_machine.userdata.BRICK_SEE_WALL = 8
    _state_machine.userdata.BRICK_SEE_WALL_HAVING_RED = 9
    _state_machine.userdata.BRICK_SEE_WALL_HAVING_GREEN = 10
    _state_machine.userdata.BRICK_SEE_WALL_HAVING_BLUE = 11
    _state_machine.userdata.LAYER_0 = 0
    _state_machine.userdata.LAYER_1 = 1
    _state_machine.userdata.wall_layer = 0
    _state_machine.userdata.flying_altitude = 3
    _state_machine.userdata.fly_with_brick_yaw = 0
    _state_machine.userdata.drop_wait_position = 0
    _state_machine.userdata.desired_yaw = 0
    _state_machine.userdata.frame_id = "gps_origin"
    _state_machine.userdata.brick_id = 0
    _state_machine.userdata.altitude_above_wall = 4
    _state_machine.userdata.max_time_ungrasped = 1
    _state_machine.userdata.brick_see_wall_typed = 8
    _state_machine.userdata.brick_placed_position = None
    _state_machine.userdata.next_to_wall_wait_position = None

    # Additional creation code can be added inside the following tags
    # [MANUAL_CREATE]
    
    param_prefix_namespace = "flexbe_behavior_launcher/";
    flying_altitude_list = rospy.get_param(param_prefix_namespace + 'main/flying_altitudes')
    uav_name_list = rospy.get_param(param_prefix_namespace + 'main/robot_name_list')
    robot_name = rospy.get_param(param_prefix_namespace + 'robot_name')
    _state_machine.userdata.robot_name = robot_name
    Logger.loginfo("[wall_ch_place_brick]: robot_name is %s" % robot_name)
    
    brick_detection_type_service_topic = rospy.get_param(param_prefix_namespace + 'main/brick_detection_type_service_topic')
    brick_detection_type_service_topic = "/" + robot_name + "/" + brick_detection_type_service_topic
    Logger.loginfo("[wall_ch_place_brick]: brick_detection_type_service_topic: %s" % brick_detection_type_service_topic)
    
    wall_layer_service_topic = rospy.get_param(param_prefix_namespace + 'main/wall_layer_service_topic')
    wall_layer_service_topic = "/" + robot_name + "/" + wall_layer_service_topic
    Logger.loginfo("[wall_ch_place_brick]: wall_layer_service_topic: %s" % wall_layer_service_topic)
    
    goto_altitude_service_topic = rospy.get_param(param_prefix_namespace + 'main/goto_altitude_service_topic')
    goto_altitude_service_topic = "/" + robot_name + "/" + goto_altitude_service_topic
    Logger.loginfo("[wall_ch_place_brick]: goto_altitude_service_topic: %s" % goto_altitude_service_topic)
    
    control_manager_diagnostics_topic = rospy.get_param(param_prefix_namespace + 'main/control_manager_diagnostics_topic')
    control_manager_diagnostics_topic = "/" + robot_name + "/" + control_manager_diagnostics_topic
    Logger.loginfo("[wall_ch_place_brick]: control_manager_diagnostics_topic: %s" % control_manager_diagnostics_topic)
    
    goto_reference_service_topic = rospy.get_param(param_prefix_namespace + 'main/goto_reference_service_topic')
    goto_reference_service_topic = "/" + robot_name + "/" + goto_reference_service_topic
    Logger.loginfo("[wall_ch_place_brick]: goto_reference_service_topic: %s" % goto_reference_service_topic)
    
    set_yaw_service_topic = rospy.get_param(param_prefix_namespace + 'main/set_yaw_service_topic')
    set_yaw_service_topic = "/" + robot_name + "/" + set_yaw_service_topic
    Logger.loginfo("[wall_ch_place_brick]: set_yaw_service_topic: %s" % set_yaw_service_topic)
    
    plan_keeper_placed_brick_in_topic = rospy.get_param(param_prefix_namespace + 'main/plan_keeper_placed_brick_in_topic')
    plan_keeper_placed_brick_in_topic = "/" + robot_name + "/" + plan_keeper_placed_brick_in_topic
    Logger.loginfo("[wall_ch_place_brick]: plan_keeper_placed_brick_in_topic: %s" % plan_keeper_placed_brick_in_topic)
    
    gripper_diagnostics_topic = rospy.get_param(param_prefix_namespace + 'main/gripper_diagnostics_topic')
    gripper_diagnostics_topic = "/" + robot_name + "/" + gripper_diagnostics_topic
    Logger.loginfo("[wall_ch_place_brick]: gripper_diagnostics_topic: %s" % gripper_diagnostics_topic)
    
    odometry_odom_gps_topic = rospy.get_param(param_prefix_namespace + 'main/odometry_odom_gps_topic')
    odometry_odom_gps_topic = "/" + robot_name + "/" + odometry_odom_gps_topic
    Logger.loginfo("[wall_ch_place_brick]: odometry_odom_gps_topic: %s" % odometry_odom_gps_topic)

    # [/MANUAL_CREATE]


    with _state_machine:
      # x:206 y:51
      OperatableStateMachine.add('SetSeeWallAndLayer',
                    ServiceSetBrickDetectionTypeAndWallLayer(service_topic_see_type=brick_detection_type_service_topic, service_topic_layer=wall_layer_service_topic),
                    transitions={'successed': 'Fly_To_Fly_Altitude', 'failed': 'Fly_To_Fly_Altitude'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'see_type': 'brick_see_wall_typed', 'wall_layer': 'wall_layer'})

      # x:971 y:619
      OperatableStateMachine.add('PlaceBrick',
                    BrickPlacingActionState(),
                    transitions={'placed': 'FlyToAltitudeAfterPlacing', 'placing_error': 'placing_badly'},
                    autonomy={'placed': Autonomy.Off, 'placing_error': Autonomy.Off},
                    remapping={'placing_position': 'placing_position', 'brick_type': 'brick_type', 'return_altitude': 'flying_altitude', 'placing_result': 'placing_result'})

      # x:526 y:85
      OperatableStateMachine.add('Fly_To_Fly_Altitude',
                    ServiceGoToAltitudeState(service_topic=goto_altitude_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'RotateRelativeToTarget', 'failed': 'failed'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal': 'flying_altitude'})

      # x:951 y:57
      OperatableStateMachine.add('RotateRelativeToTarget',
                    ServiceRotateRelativeToGoToState(set_yaw_service_topic=set_yaw_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic, odometry_topic=odometry_odom_gps_topic),
                    transitions={'successed': 'IsStillGripped2', 'failed': 'RotateRelativeToTarget'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal_tracker_point': 'drop_wait_position', 'frame_id': 'frame_id', 'yaw_relative_to_goal': 'fly_with_brick_yaw', 'desired_yaw': 'desired_yaw'})

      # x:1089 y:403
      OperatableStateMachine.add('FlyToDropWaitPosition',
                    ServiceGoToReferenceState(service_topic=goto_reference_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'IsStillGripped', 'failed': 'failed'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal_tracker_point': 'drop_wait_position', 'frame_id': 'frame_id'})

      # x:380 y:657
      OperatableStateMachine.add('SetPlacedBrick',
                    ServiceSetBrickPlacedIntoPlanKeeper(service_topic=plan_keeper_placed_brick_in_topic),
                    transitions={'successed': 'finished', 'failed': 'finished'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'brick_id': 'brick_id', 'placing_result': 'placing_result'})

      # x:1185 y:518
      OperatableStateMachine.add('FlyToAltitudeAboveWall',
                    ServiceGoToAltitudeState(service_topic=goto_altitude_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'PlaceBrick', 'failed': 'FlyToAltitudeAboveWall'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal': 'altitude_above_wall'})

      # x:699 y:696
      OperatableStateMachine.add('FlyToAltitudeAfterPlacing',
                    ServiceGoToAltitudeState(service_topic=goto_altitude_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'SetPlacedBrick', 'failed': 'FlyToAltitudeAfterPlacing'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal': 'flying_altitude'})

      # x:939 y:501
      OperatableStateMachine.add('IsStillGripped',
                    IsGraspedState(gripper_diagnostics_topic=gripper_diagnostics_topic),
                    transitions={'is_connected': 'FlyToAltitudeAboveWall', 'unconnected': 'dropped_brick'},
                    autonomy={'is_connected': Autonomy.Off, 'unconnected': Autonomy.Off},
                    remapping={'max_time_ungrasped': 'max_time_ungrasped'})

      # x:1057 y:159
      OperatableStateMachine.add('IsStillGripped2',
                    IsGraspedState(gripper_diagnostics_topic=gripper_diagnostics_topic),
                    transitions={'is_connected': 'FlyNextToWallWithDesiredYaw', 'unconnected': 'dropped_brick'},
                    autonomy={'is_connected': Autonomy.Off, 'unconnected': Autonomy.Off},
                    remapping={'max_time_ungrasped': 'max_time_ungrasped'})

      # x:1277 y:285
      OperatableStateMachine.add('FlyNextToWallWaitPosition',
                    ServiceGoToReferenceState(service_topic=goto_reference_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'FlyToDropWaitPosition', 'failed': 'FlyNextToWallWaitPosition'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal_tracker_point': 'drop_wait_position', 'frame_id': 'frame_id'})

      # x:931 y:242
      OperatableStateMachine.add('FlyNextToWallWithDesiredYaw',
                    ServiceGoToReferenceWithDesiredYawState(service_topic=goto_reference_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'FlyNextToWallWaitPosition', 'failed': 'FlyNextToWallWithDesiredYaw'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal_tracker_point': 'next_to_wall_wait_position', 'desired_yaw': 'desired_yaw', 'frame_id': 'frame_id'})


    return _state_machine


  # Private functions can be added inside the following tags
  # [MANUAL_FUNC]
  
  # [/MANUAL_FUNC]
