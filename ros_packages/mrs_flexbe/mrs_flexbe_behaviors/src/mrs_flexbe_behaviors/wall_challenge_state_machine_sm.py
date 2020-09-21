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
from mrs_flexbe_states.service_next_brick_in_plan import ServiceGetNextBrickInPlan
from mrs_flexbe_states.brick_grasping_action_state import BrickGraspingActionState
from mrs_flexbe_behaviors.scan_arena_sm import scan_arenaSM
from mrs_flexbe_states.service_goto_altitude_state import ServiceGoToAltitudeState
from mrs_flexbe_states.service_set_mapped_objects_into_plan_keeper import ServiceSetMappedObjectsIntoPlanKeeper
from mrs_flexbe_states.service_goto_reference_state import ServiceGoToReferenceState
from mrs_flexbe_states.is_battery_bellow import IsBatteryBellowValueState
from mrs_flexbe_behaviors.takeoff_sm import TakeOffSM
from mrs_flexbe_behaviors.wall_ch_place_brick_sm import wall_ch_place_brickSM
from mrs_flexbe_states.is_realsense_connected import IsRealsenseConnectedState
from flexbe_states.decision_state import DecisionState
from mrs_flexbe_states.wait_for_arena_mapped_objects import WaitForArenaMappedObjectsState
from mrs_flexbe_states.wait_for_other_drone_end import WaitForOtherDronesEnd
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import rospy
import mrs_msgs.msg
# [/MANUAL_IMPORT]


'''
Created on Tue Sep 17 2019
@author: Vojtech Spurny, Robert Penicka
'''
class wall_challenge_state_machineSM(Behavior):
  '''
  Main state machine for the second challenge of MBZIRC 2020
  '''


  def __init__(self):
    super(wall_challenge_state_machineSM, self).__init__()
    self.name = 'wall_challenge_state_machine'

    # parameters of this behavior

    # references to used behaviors
    self.add_behavior(scan_arenaSM, 'scan_arena')
    self.add_behavior(TakeOffSM, 'TakeOff')
    self.add_behavior(wall_ch_place_brickSM, 'wall_ch_place_brick')

    # Additional initialization code can be added inside the following tags
    # [MANUAL_INIT]
    
    # [/MANUAL_INIT]

    # Behavior comments:



  def create(self):
    # x:475 y:443, x:417 y:374
    _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
    _state_machine.userdata.num_robots = 3
    _state_machine.userdata.remaining_time_s = 500
    _state_machine.userdata.max_planning_time_s = 30
    _state_machine.userdata.times_finish_actual_s = [0, 0, 0]
    _state_machine.userdata.builded_brick_ids = []
    _state_machine.userdata.simulation = True
    _state_machine.userdata.robot_name = "uav1"
    _state_machine.userdata.flying_velocity = 2
    _state_machine.userdata.scanning_robots = 1
    _state_machine.userdata.scanning_altitude = 8.0
    _state_machine.userdata.scanning_speed = 5.0
    _state_machine.userdata.scanning_max_acc = 2.0
    _state_machine.userdata.scanning_turning_speed = 2.0
    _state_machine.userdata.robot_id = 0
    _state_machine.userdata.flying_altitude = 3
    _state_machine.userdata.frame_id = "gps_origin"
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
    _state_machine.userdata.fly_with_brick_yaw = 0
    _state_machine.userdata.cell_voltage_do_not_grasp = 3.8
    _state_machine.userdata.max_time_unconnected_realsense_s = 3
    _state_machine.userdata.mapped_objects = None
    _state_machine.userdata.altitude_above_wall = 4

    # Additional creation code can be added inside the following tags
    # [MANUAL_CREATE]
    
    #option to change them manually

    param_prefix_namespace = "flexbe_behavior_launcher/";
    
    flying_altitude_list = rospy.get_param(param_prefix_namespace + 'main/flying_altitudes')
    uav_name_list = rospy.get_param(param_prefix_namespace + 'main/robot_name_list')
    robot_name = rospy.get_param(param_prefix_namespace + 'robot_name')
    _state_machine.userdata.robot_name = robot_name
    Logger.loginfo("[wall_challenge_state_machine]: robot_name is %s"% robot_name)
    frame_id = rospy.get_param(param_prefix_namespace + 'main/frame_id')
    battery_num_cells = rospy.get_param(param_prefix_namespace + 'main/battery_num_cells')
    _state_machine.userdata.cell_voltage_do_not_grasp = rospy.get_param(param_prefix_namespace + 'main/cell_voltage_do_not_grasp')
    
    clempeeren_wait_after_start_s = rospy.get_param(param_prefix_namespace + 'main/clempeeren_wait_after_start_s')
    clempeeren_not_receiving_other_diagnostics_s = rospy.get_param(param_prefix_namespace + 'main/clempeeren_not_receiving_other_diagnostics_s')
    
    _state_machine.userdata.altitude_above_wall = rospy.get_param(param_prefix_namespace + 'main/altitude_above_wall')
    Logger.loginfo("[wall_challenge_state_machine]: altitude_above_wall is %s"% _state_machine.userdata.altitude_above_wall)
    
    first_robot_name = uav_name_list[0]
    Logger.loginfo("[wall_challenge_state_machine]: first_robot_name is %s"% first_robot_name)
    
    _state_machine.userdata.flying_altitude = 3
    for i in range(len(uav_name_list)):
      if uav_name_list[i] == robot_name:
        _state_machine.userdata.flying_altitude = flying_altitude_list[i]
        flying_altitude = _state_machine.userdata.flying_altitude
        _state_machine.userdata.robot_id = i
        break
    
    grasp_return_altitude = rospy.get_param(param_prefix_namespace + 'main/grasp_return_altitude')
    Logger.loginfo("[wall_challenge_state_machine]: grasp_return_altitude is %s"% (str(grasp_return_altitude)))
    
    #arena scanning params part begin
    _state_machine.userdata.scanning_speed = rospy.get_param(param_prefix_namespace + 'arena_scanning/scanning_speed')
    Logger.loginfo("[wall_challenge_state_machine]: loaded arena_scanning/scanning_speed: %s" % (str(_state_machine.userdata.scanning_speed)))

    _state_machine.userdata.scanning_altitude = rospy.get_param(param_prefix_namespace + 'arena_scanning/scanning_altitude')
    Logger.loginfo("[wall_challenge_state_machine]: loaded arena_scanning/scanning_altitude: %s" % (str(_state_machine.userdata.scanning_altitude)))

    _state_machine.userdata.scanning_max_acc = rospy.get_param(param_prefix_namespace + 'arena_scanning/scanning_max_acc')
    Logger.loginfo("[wall_challenge_state_machine]: loaded arena_scanning/scanning_max_acc: %s" % (str(_state_machine.userdata.scanning_max_acc)))

    _state_machine.userdata.scanning_turning_speed = rospy.get_param(param_prefix_namespace + 'arena_scanning/scanning_turning_speed')
    Logger.loginfo("[wall_challenge_state_machine]: loaded arena_scanning/scanning_turning_speed: %s" % (str(_state_machine.userdata.scanning_turning_speed)))
    
    _state_machine.userdata.scanning_robots = rospy.get_param(param_prefix_namespace + 'arena_scanning/scanning_robots')
    Logger.loginfo("[wall_challenge_state_machine]: loaded arena_scanning/scanning_robots: %s" % (str(_state_machine.userdata.scanning_robots)))

    _state_machine.userdata.fly_with_brick_yaw = rospy.get_param(param_prefix_namespace + 'main/fly_with_brick_yaw')

    realsense_camera_info_topic = rospy.get_param(param_prefix_namespace + 'main/realsense_camera_info_topic')
    realsense_camera_info_topic = "/" + robot_name + "/" + realsense_camera_info_topic
    Logger.loginfo("[wall_challenge_state_machine]: realsense_camera_info_topic: %s" % realsense_camera_info_topic)
    

    plan_keeper_plan_in_topic = rospy.get_param(param_prefix_namespace + 'main/plan_keeper_plan_in_topic')
    plan_keeper_plan_in_topic = "/" + robot_name + "/" + plan_keeper_plan_in_topic
    Logger.loginfo("[wall_challenge_state_machine]: plan_keeper_plan_in_topic: %s" % plan_keeper_plan_in_topic)
    
    plan_keeper_get_brick_plan_topic = rospy.get_param(param_prefix_namespace + 'main/plan_keeper_get_brick_plan_topic')
    plan_keeper_get_brick_plan_topic = "/" + robot_name + "/" + plan_keeper_get_brick_plan_topic
    Logger.loginfo("[wall_challenge_state_machine]: plan_keeper_get_brick_plan_topic: %s" % plan_keeper_get_brick_plan_topic)

    plan_keeper_placed_brick_in_topic = rospy.get_param(param_prefix_namespace + 'main/plan_keeper_placed_brick_in_topic')
    plan_keeper_placed_brick_in_topic = "/" + robot_name + "/" + plan_keeper_placed_brick_in_topic
    Logger.loginfo("[wall_challenge_state_machine]: plan_keeper_placed_brick_in_topic: %s" % plan_keeper_placed_brick_in_topic)

    set_mapped_objects_to_plan_keeper_service_topic = rospy.get_param(param_prefix_namespace + 'main/set_mapped_objects_to_plan_keeper_service_topic')
    set_mapped_objects_to_plan_keeper_service_topic = "/" + robot_name + "/" + set_mapped_objects_to_plan_keeper_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: set_mapped_objects_to_plan_keeper_service_topic: %s" % set_mapped_objects_to_plan_keeper_service_topic)

    wall_definition_service_topic = rospy.get_param(param_prefix_namespace + 'main/wall_definition_service_topic')
    wall_definition_service_topic = "/" + robot_name + "/" + wall_definition_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: wall_definition_service_topic: %s" % wall_definition_service_topic)
    
    building_rules_service_topic = rospy.get_param(param_prefix_namespace + 'main/building_rules_service_topic')
    building_rules_service_topic = "/" + robot_name + "/" + building_rules_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: building_rules_service_topic: %s" % building_rules_service_topic)
    
    plan_keeper_wall_definition_in_service_topic = rospy.get_param(param_prefix_namespace + 'main/plan_keeper_wall_definition_in_service_topic')
    plan_keeper_wall_definition_in_service_topic = "/" + robot_name + "/" + plan_keeper_wall_definition_in_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: plan_keeper_wall_definition_in_service_topic: %s" % plan_keeper_wall_definition_in_service_topic)
    
    plan_keeper_building_rules_in_service_topic = rospy.get_param(param_prefix_namespace + 'main/plan_keeper_building_rules_in_service_topic')
    plan_keeper_building_rules_in_service_topic = "/" + robot_name + "/" + plan_keeper_building_rules_in_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: plan_keeper_building_rules_in_service_topic: %s" % plan_keeper_building_rules_in_service_topic)
    
    plan_keeper_is_brick_placeable_service_topic = rospy.get_param(param_prefix_namespace + 'main/plan_keeper_is_brick_placeable_service_topic')
    plan_keeper_is_brick_placeable_service_topic = "/" + robot_name + "/" + plan_keeper_is_brick_placeable_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: plan_keeper_is_brick_placeable_service_topic: %s" % plan_keeper_is_brick_placeable_service_topic)

    land_home_service_topic = rospy.get_param(param_prefix_namespace + 'main/land_home_service_topic')
    land_home_service_topic = "/" + robot_name + "/" + land_home_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: land_home_service_topic: %s" % land_home_service_topic)

    flyto_server_name = rospy.get_param(param_prefix_namespace + 'main/flyto_server_name')
    flyto_server_name = "/" + robot_name + "/" + flyto_server_name
    Logger.loginfo("[wall_challenge_state_machine]: flyto_server_name: %s" % flyto_server_name)

    goto_service_topic = rospy.get_param(param_prefix_namespace + 'main/goto_service_topic')
    goto_service_topic = "/" + robot_name + "/" + goto_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: goto_service_topic: %s" % goto_service_topic)
    
    goto_reference_service_topic = rospy.get_param(param_prefix_namespace + 'main/goto_reference_service_topic')
    goto_reference_service_topic = "/" + robot_name + "/" + goto_reference_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: goto_reference_service_topic: %s" % goto_reference_service_topic)
    
    goto_altitude_service_topic = rospy.get_param(param_prefix_namespace + 'main/goto_altitude_service_topic')
    goto_altitude_service_topic = "/" + robot_name + "/" + goto_altitude_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: goto_altitude_service_topic: %s" % goto_altitude_service_topic)
    
    set_yaw_service_topic = rospy.get_param(param_prefix_namespace + 'main/set_yaw_service_topic')
    set_yaw_service_topic = "/" + robot_name + "/" + set_yaw_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: set_yaw_service_topic: %s" % set_yaw_service_topic)
    
    control_manager_diagnostics_topic = rospy.get_param(param_prefix_namespace + 'main/control_manager_diagnostics_topic')
    control_manager_diagnostics_topic = "/" + robot_name + "/" + control_manager_diagnostics_topic
    Logger.loginfo("[wall_challenge_state_machine]: control_manager_diagnostics_topic: %s" % control_manager_diagnostics_topic)
    
    shared_diagnostics_topic = rospy.get_param(param_prefix_namespace + 'main/shared_diagnostics_topic')
    Logger.loginfo("[wall_challenge_state_machine]: shared_diagnostics_topic: %s" % shared_diagnostics_topic)
    
    wait_for_start_service_topic = rospy.get_param(param_prefix_namespace + 'main/wait_for_start_service_topic')
    wait_for_start_service_topic = "/" + robot_name + "/" + wait_for_start_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: wait_for_start_service_topic: %s" % wait_for_start_service_topic)
    
    brick_detection_type_service_topic = rospy.get_param(param_prefix_namespace + 'main/brick_detection_type_service_topic')
    brick_detection_type_service_topic = "/" + robot_name + "/" + brick_detection_type_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: brick_detection_type_service_topic: %s" % brick_detection_type_service_topic)
    
    wall_layer_service_topic = rospy.get_param(param_prefix_namespace + 'main/wall_layer_service_topic')
    wall_layer_service_topic = "/" + robot_name + "/" + wall_layer_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: wall_layer_service_topic: %s" % wall_layer_service_topic)
    
    gripper_ungrip_service_topic = rospy.get_param(param_prefix_namespace + 'main/gripper_ungrip_service_topic')
    gripper_ungrip_service_topic = "/" + robot_name + "/" + gripper_ungrip_service_topic
    Logger.loginfo("[wall_challenge_state_machine]: gripper_ungrip_service_topic: %s" % gripper_ungrip_service_topic) 
    
    battery_status_topic = rospy.get_param(param_prefix_namespace + 'main/battery_status_topic')
    battery_status_topic = "/" + robot_name + "/" + battery_status_topic
    Logger.loginfo("[wall_challenge_state_machine]: battery_status_topic: %s" % battery_status_topic)
    
    e_land_topic = rospy.get_param(param_prefix_namespace + 'main/e_land_topic')
    e_land_topic = "/" + robot_name + "/" + e_land_topic
    Logger.loginfo("[wall_challenge_state_machine]: e_land_topic: %s" % e_land_topic)
    
    load_mapped_arena_from_file_topic = rospy.get_param(param_prefix_namespace + 'main/load_mapped_arena_from_file_topic')
    load_mapped_arena_from_file_topic = "/" + robot_name + "/" + load_mapped_arena_from_file_topic
    Logger.loginfo("[wall_challenge_state_machine]: load_mapped_arena_from_file_topic: %s" % load_mapped_arena_from_file_topic)
    
    Logger.loginfo("[wall_challenge_state_machine]: first_robot_name %s is used for getting mapped arena topic: " % first_robot_name)
    plan_keeper_mapped_arena_topic = rospy.get_param(param_prefix_namespace + 'main/plan_keeper_mapped_arena_topic')
    plan_keeper_mapped_arena_topic = "/" + first_robot_name + "/" + plan_keeper_mapped_arena_topic
    Logger.loginfo("[wall_challenge_state_machine]: plan_keeper_mapped_arena_topic: %s" % plan_keeper_mapped_arena_topic)
    
    brick_estimation_stop_start_map_service = rospy.get_param(param_prefix_namespace + 'main/brick_estimation_stop_start_map_service')
    brick_estimation_stop_start_map_service = "/" + robot_name + "/" + brick_estimation_stop_start_map_service
    Logger.loginfo("[wall_challenge_state_machine]: brick_estimation_stop_start_map_service: %s" % brick_estimation_stop_start_map_service)
    
    
    brick_estimation_closest_wall_topic = rospy.get_param(param_prefix_namespace + 'main/brick_estimation_closest_wall_topic')
    brick_estimation_closest_wall_topic = "/" + robot_name + "/" + brick_estimation_closest_wall_topic
    Logger.loginfo("[wall_challenge_state_machine]: brick_estimation_closest_wall_topic: %s" % brick_estimation_closest_wall_topic)
    
    
    drop_start_altitude = rospy.get_param(param_prefix_namespace + 'main/drop_start_altitude')
    
    Logger.loginfo("[wall_challenge_state_machine]: wall_ch_sm.robot_name: %s" % robot_name)
    Logger.loginfo("[wall_challenge_state_machine]: wall_ch_sm.remaining_time_s: %s" % _state_machine.userdata.remaining_time_s)
    Logger.loginfo("[wall_challenge_state_machine]: wall_ch_sm.max_planning_time_s: %s" % _state_machine.userdata.max_planning_time_s)
    Logger.loginfo("[wall_challenge_state_machine]: wall_ch_sm.drop_start_altitude: %s" % drop_start_altitude)
    

    
    Logger.loginfo("[wall_challenge_state_machine]: robot_id: %d" % _state_machine.userdata.robot_id)
    Logger.loginfo("[wall_challenge_state_machine]: wall_ch_sm.flying_altitude: %s" % flying_altitude)
          
    _state_machine.userdata.num_robots = len(uav_name_list)
    Logger.loginfo("[wall_challenge_state_machine]: wall_ch_sm.num_robots: %s" % _state_machine.userdata.num_robots)
    
    _state_machine.userdata.times_finish_actual_s = []
    for i in range(len(uav_name_list)):
      _state_machine.userdata.times_finish_actual_s.append(0)
      
    other_diagnostics_topics = []
    for i in range(len(uav_name_list)):
      if uav_name_list[i] != robot_name:
        other_r_shared_diagnostics_topic = "/" + uav_name_list[i] + "/" + shared_diagnostics_topic
        other_diagnostics_topics.append(other_r_shared_diagnostics_topic)

    Logger.loginfo("[wall_challenge_state_machine]: other_diagnostics_topics: %s" % str(other_diagnostics_topics))    
    Logger.loginfo("[wall_challenge_state_machine]: wall_ch_sm.num_robots: %s" % _state_machine.userdata.num_robots)
    
    # rospy.loginfo(robot_name)
    
    # [/MANUAL_CREATE]


    with _state_machine:
      # x:35 y:120
      OperatableStateMachine.add('WaitForStart',
                    ServiceWaitForStart(service_topic=wait_for_start_service_topic),
                    transitions={'received_one': 'IsScanningDrone1', 'received_two': 'IsClempeeringDrone1', 'received_three': 'IsClempeeringDrone1', 'error': 'failed'},
                    autonomy={'received_one': Autonomy.Off, 'received_two': Autonomy.Off, 'received_three': Autonomy.Off, 'error': Autonomy.Off},
                    remapping={'start_value': 'start_value'})

      # x:669 y:560
      OperatableStateMachine.add('Call_Land_Home',
                    ServiceTriggerState(service_topic=land_home_service_topic, state_name=None),
                    transitions={'finished': 'finished', 'failed': 'failed'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:709 y:145
      OperatableStateMachine.add('Get_Next_Brick_Plan',
                    ServiceGetNextBrickInPlan(service_topic=plan_keeper_get_brick_plan_topic, flying_altitude=flying_altitude),
                    transitions={'successed': 'Fly_To_ Fly_Grasp_Altitude', 'finished': 'FlyToFlyAltitudeBeforeLandHome', 'failed': 'FlyToFlyAltitudeBeforeLandHome'},
                    autonomy={'successed': Autonomy.Off, 'finished': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'grasp_position': 'grasp_position', 'placing_position': 'placing_position', 'drop_wait_position': 'drop_wait_position', 'next_to_wall_wait_position': 'next_to_wall_wait_position', 'brick_type': 'brick_type', 'brick_see_wall_typed': 'brick_see_wall_typed', 'brick_id': 'brick_id', 'wall_layer': 'wall_layer'})

      # x:1406 y:251
      OperatableStateMachine.add('Grasp_Brick',
                    BrickGraspingActionState(return_altitude=grasp_return_altitude),
                    transitions={'grasped': 'wall_ch_place_brick', 'grasping_error': 'Get_Next_Brick_Plan'},
                    autonomy={'grasped': Autonomy.Off, 'grasping_error': Autonomy.Off},
                    remapping={'brick_type': 'brick_type'})

      # x:118 y:43
      OperatableStateMachine.add('scan_arena',
                    self.use_behavior(scan_arenaSM, 'scan_arena'),
                    transitions={'finished': 'Set_Mapped_Objects_Into_Plan_Keeper', 'failed': 'FlyToFlyAltitudeBeforeLandHome'},
                    autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                    remapping={'scanning_speed': 'scanning_speed', 'scanning_robots': 'scanning_robots', 'scanning_altitude': 'scanning_altitude', 'scanning_max_acc': 'scanning_max_acc', 'scanning_turning_speed': 'scanning_turning_speed', 'robot_id': 'robot_id', 'mapped_objects': 'mapped_objects'})

      # x:720 y:19
      OperatableStateMachine.add('Fly_To_ Fly_Grasp_Altitude',
                    ServiceGoToAltitudeState(service_topic=goto_altitude_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'Fly_To_Grasp_position', 'failed': 'Fly_To_ Fly_Grasp_Altitude'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal': 'flying_altitude'})

      # x:414 y:37
      OperatableStateMachine.add('Set_Mapped_Objects_Into_Plan_Keeper',
                    ServiceSetMappedObjectsIntoPlanKeeper(service_topic=set_mapped_objects_to_plan_keeper_service_topic),
                    transitions={'successed': 'Get_Next_Brick_Plan', 'failed': 'FlyToFlyAltitudeBeforeLandHome'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'mapped_objects': 'mapped_objects'})

      # x:1052 y:73
      OperatableStateMachine.add('Fly_To_Grasp_position',
                    ServiceGoToReferenceState(service_topic=goto_reference_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'CheckBatteryBeforeGrasping', 'failed': 'Fly_To_Grasp_position'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal_tracker_point': 'grasp_position', 'frame_id': 'frame_id'})

      # x:1367 y:59
      OperatableStateMachine.add('CheckBatteryBeforeGrasping',
                    IsBatteryBellowValueState(battery_status_topic=battery_status_topic, num_battery_cells=battery_num_cells),
                    transitions={'is_bellow': 'FlyToFlyAltitudeBeforeLandHome', 'is_ok': 'CheckRealsenseBeforeGrasping'},
                    autonomy={'is_bellow': Autonomy.Off, 'is_ok': Autonomy.Off},
                    remapping={'cell_voltage_target': 'cell_voltage_do_not_grasp', 'current_cell_voltage': 'current_cell_voltage'})

      # x:249 y:524
      OperatableStateMachine.add('TakeOff',
                    self.use_behavior(TakeOffSM, 'TakeOff'),
                    transitions={'finished': 'FlyToFlyAltitudeTakeOff', 'failed': 'Emergency_landing'},
                    autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                    remapping={'simulation': 'simulation'})

      # x:732 y:790
      OperatableStateMachine.add('Emergency_landing',
                    ServiceTriggerState(service_topic=e_land_topic, state_name="Emergency_landing"),
                    transitions={'finished': 'finished', 'failed': 'failed'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:1390 y:508
      OperatableStateMachine.add('wall_ch_place_brick',
                    self.use_behavior(wall_ch_place_brickSM, 'wall_ch_place_brick'),
                    transitions={'finished': 'Get_Next_Brick_Plan', 'failed': 'FlyToFlyAltitudeBeforeLandHome', 'placing_badly': 'Fly_To_Grasp_position', 'dropped_brick': 'Fly_To_Grasp_position'},
                    autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'placing_badly': Autonomy.Inherit, 'dropped_brick': Autonomy.Inherit},
                    remapping={'brick_type': 'brick_type', 'placing_position': 'placing_position', 'wall_layer': 'wall_layer', 'flying_altitude': 'flying_altitude', 'fly_with_brick_yaw': 'fly_with_brick_yaw', 'drop_wait_position': 'drop_wait_position', 'brick_id': 'brick_id', 'altitude_above_wall': 'altitude_above_wall', 'brick_see_wall_typed': 'brick_see_wall_typed', 'brick_placed_position': 'drop_wait_position', 'next_to_wall_wait_position': 'next_to_wall_wait_position', 'desired_yaw': 'desired_yaw'})

      # x:1551 y:144
      OperatableStateMachine.add('CheckRealsenseBeforeGrasping',
                    IsRealsenseConnectedState(realsense_camera_info_topic=realsense_camera_info_topic),
                    transitions={'is_connected': 'Grasp_Brick', 'unconnected': 'FlyToFlyAltitudeBeforeLandHome'},
                    autonomy={'is_connected': Autonomy.Off, 'unconnected': Autonomy.Off},
                    remapping={'max_time_unconnected_realsense_s': 'max_time_unconnected_realsense_s'})

      # x:310 y:175
      OperatableStateMachine.add('IsScanningDrone2',
                    DecisionState(outcomes=['is_scanning_drone', 'no_scanning_drone'], conditions=lambda x: 'is_scanning_drone' if x==0 else 'no_scanning_drone'),
                    transitions={'is_scanning_drone': 'scan_arena', 'no_scanning_drone': 'Set_Mapped_Objects_Into_Plan_Keeper'},
                    autonomy={'is_scanning_drone': Autonomy.Off, 'no_scanning_drone': Autonomy.Off},
                    remapping={'input_value': 'robot_id'})

      # x:10 y:828
      OperatableStateMachine.add('WaitForMappedObjectsStates',
                    WaitForArenaMappedObjectsState(plan_keeper_mapped_arena_topic=plan_keeper_mapped_arena_topic),
                    transitions={'successed': 'IsClempeeringDrone1', 'failed': 'Emergency_landing'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'mapped_objects': 'mapped_objects'})

      # x:7 y:540
      OperatableStateMachine.add('IsScanningDrone1',
                    DecisionState(outcomes=['is_scanning_drone', 'no_scanning_drone'], conditions=lambda x: 'is_scanning_drone' if x==0 else 'no_scanning_drone'),
                    transitions={'is_scanning_drone': 'TakeOff', 'no_scanning_drone': 'WaitForMappedObjectsStates'},
                    autonomy={'is_scanning_drone': Autonomy.Off, 'no_scanning_drone': Autonomy.Off},
                    remapping={'input_value': 'robot_id'})

      # x:902 y:530
      OperatableStateMachine.add('FlyToFlyAltitudeBeforeLandHome',
                    ServiceGoToAltitudeState(service_topic=goto_altitude_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'Call_Land_Home', 'failed': 'Call_Land_Home'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal': 'flying_altitude'})

      # x:256 y:414
      OperatableStateMachine.add('FlyToFlyAltitudeTakeOff',
                    ServiceGoToAltitudeState(service_topic=goto_altitude_service_topic, control_manager_diagnostics_topic=control_manager_diagnostics_topic),
                    transitions={'successed': 'IsAfterRestart', 'failed': 'FlyToFlyAltitudeTakeOff'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                    remapping={'goal': 'flying_altitude'})

      # x:343 y:293
      OperatableStateMachine.add('IsAfterRestart',
                    DecisionState(outcomes=['is_after_restart','fresh'], conditions=lambda x: 'fresh' if x==0 else 'is_after_restart'),
                    transitions={'is_after_restart': 'LoadMapFromFile', 'fresh': 'IsScanningDrone2'},
                    autonomy={'is_after_restart': Autonomy.Off, 'fresh': Autonomy.Off},
                    remapping={'input_value': 'start_value'})

      # x:526 y:265
      OperatableStateMachine.add('LoadMapFromFile',
                    ServiceTriggerState(service_topic=load_mapped_arena_from_file_topic, state_name=None),
                    transitions={'finished': 'Get_Next_Brick_Plan', 'failed': 'Get_Next_Brick_Plan'},
                    autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

      # x:149 y:731
      OperatableStateMachine.add('IsClempeeringDrone1',
                    DecisionState(outcomes=['is_clampeering_drone', 'not_clampeering_drone'], conditions=lambda x: 'is_clampeering_drone' if x==1 else 'not_clampeering_drone'),
                    transitions={'is_clampeering_drone': 'WaitForEndOfOtherDrones', 'not_clampeering_drone': 'TakeOff'},
                    autonomy={'is_clampeering_drone': Autonomy.Off, 'not_clampeering_drone': Autonomy.Off},
                    remapping={'input_value': 'robot_id'})

      # x:419 y:768
      OperatableStateMachine.add('WaitForEndOfOtherDrones',
                    WaitForOtherDronesEnd(other_r_shared_diagnostics_topic=other_diagnostics_topics, wait_after_start_s=clempeeren_wait_after_start_s, wait_time_s=clempeeren_not_receiving_other_diagnostics_s),
                    transitions={'successed': 'TakeOff', 'failed': 'WaitForEndOfOtherDrones'},
                    autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off})


    return _state_machine


  # Private functions can be added inside the following tags
  # [MANUAL_FUNC]
  
  # [/MANUAL_FUNC]
