#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mrs_flexbe_behaviors.prepare_uav_sm import Prepare_UAVSM
from mrs_flexbe_behaviors.goto_altitude_sm import GotoAltitudeSM
from mrs_flexbe_states.window_flyer_action_state import WindowFlyerActionState
from mrs_flexbe_states.service_trigger_state import ServiceTriggerState
from mrs_flexbe_states.wait_for_msg_state import WaitForMsgState
from flexbe_states.wait_state import WaitState
from mrs_flexbe_behaviors.goto_reference_sm import goto_referenceSM
from mrs_flexbe_states.service_setbool_state import ServiceSetBoolState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import math
import rospy
import mrs_msgs.msg
import mrs_msgs.srv
# [/MANUAL_IMPORT]


'''
Created on Tue Sep 17 2019
@author: Vojtech Spurny
'''
class fire_challenge_inside_smSM(Behavior):
    '''
    Main state machine for the third challenge of MBZIRC 2020
    '''


    def __init__(self):
        super(fire_challenge_inside_smSM, self).__init__()
        self.name = 'fire_challenge_inside_sm'

        # parameters of this behavior

        # references to used behaviors
        self.add_behavior(Prepare_UAVSM, 'Prepare_UAV')
        self.add_behavior(GotoAltitudeSM, 'GotoAltitude')
        self.add_behavior(goto_referenceSM, 'Goto_Waiting_Spot')

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

    # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        wait_time_after_flying = 1
        wall_following_desired_yaw = -math.pi/2
        # x:141 y:643, x:263 y:340
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.simulation = True
        _state_machine.userdata.flying_altitude = 5
        _state_machine.userdata.waiting_spot = mrs_msgs.srv.ReferenceStampedSrvRequest()

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        param_prefix_namespace = "flexbe_behavior_launcher/";
        flying_altitude_list = rospy.get_param(param_prefix_namespace + 'main/flying_altitudes')
        uav_name_list = rospy.get_param(param_prefix_namespace + 'main/robot_name_list')
        robot_name = rospy.get_param(param_prefix_namespace + 'robot_name')
        waiting_spots = rospy.get_param(param_prefix_namespace + 'main/waiting_spots')
        wall_following_desired_speed = rospy.get_param(param_prefix_namespace + 'main/wall_following_desired_speed')
        wall_following_desired_wall_distance = rospy.get_param(param_prefix_namespace + 'main/wall_following_desired_wall_distance')
        _state_machine.userdata.robot_name = robot_name

        Logger.loginfo("wall_following_desired_speed: %s" % wall_following_desired_speed)
        Logger.loginfo("wall_following_desired_wall_distance: %s" % wall_following_desired_wall_distance)

        window_position_estimation_reset_topic = rospy.get_param(param_prefix_namespace + 'main/window_position_estimation_reset_topic')
        window_position_estimation_reset_topic = "/" + robot_name + "/" + window_position_estimation_reset_topic
        Logger.loginfo("window_position_estimation_reset_topic: %s" % window_position_estimation_reset_topic)

        window_position_estimation_closest_window_topic = rospy.get_param(param_prefix_namespace + 'main/window_position_estimation_closest_window_topic')
        window_position_estimation_closest_window_topic = "/" + robot_name + "/" + window_position_estimation_closest_window_topic
        Logger.loginfo("window_position_estimation_closest_window_topic: %s" % window_position_estimation_closest_window_topic)

        land_home_service_topic = rospy.get_param(param_prefix_namespace + 'main/land_home_service_topic')
        land_home_service_topic = "/" + robot_name + "/" + land_home_service_topic
        Logger.loginfo("land_home_service_topic: %s" % land_home_service_topic)
        
        fire_detect_set_realsense_mode_topic = rospy.get_param(param_prefix_namespace + 'main/fire_detect_set_realsense_mode_topic')
        fire_detect_set_realsense_mode_topic = "/" + robot_name + "/" + fire_detect_set_realsense_mode_topic
        Logger.loginfo("land_home_service_topic: %s" % fire_detect_set_realsense_mode_topic)

        e_land_topic = rospy.get_param(param_prefix_namespace + 'main/e_land_topic')
        e_land_topic = "/" + robot_name + "/" + e_land_topic
        Logger.loginfo("e_land_topic: %s" % e_land_topic)

        fire_detection_topic = rospy.get_param(param_prefix_namespace + 'main/fire_detection_topic')
        fire_detection_topic = "/" + robot_name + "/" + fire_detection_topic
        Logger.loginfo("fire_detection_topic: %s" % fire_detection_topic)

        window_flyer_action_server_name = rospy.get_param(param_prefix_namespace + 'main/window_flyer_action_server_name')
        window_flyer_action_server_name = "/" + robot_name + "/" + window_flyer_action_server_name
        Logger.loginfo("window_flyer_action_server_name: %s" % window_flyer_action_server_name)

        wall_following_action_server_name = rospy.get_param(param_prefix_namespace + 'main/wall_following_action_server_name')
        wall_following_action_server_name = "/" + robot_name + "/" + wall_following_action_server_name
        Logger.loginfo("wall_following_action_server_name: %s" % wall_following_action_server_name)

        Logger.loginfo("fire_ch_sm.robot_name: %s" % robot_name)

        flying_altitude = 3
        waiting_spot = mrs_msgs.srv.ReferenceStampedSrvRequest()
        waiting_spot.header.frame_id = robot_name + "/rtk_origin"
        for i in range(len(uav_name_list)):
            if uav_name_list[i] == robot_name:
                flying_altitude = flying_altitude_list[i]
            waiting_spot.reference.position.x = waiting_spots[i * 3]
            waiting_spot.reference.position.y = waiting_spots[i * 3 + 1]
            waiting_spot.reference.position.z = flying_altitude
            waiting_spot.reference.yaw = waiting_spots[i * 3 + 2]
            break

        _state_machine.userdata.flying_altitude = flying_altitude 
        _state_machine.userdata.waiting_spot = waiting_spot 

        Logger.loginfo("fire_ch_sm.flying_altitude: %s" % _state_machine.userdata.flying_altitude)
        Logger.loginfo("fire_ch_sm.waiting_spot: %s" % _state_machine.userdata.waiting_spot)

    # [/MANUAL_CREATE]


        with _state_machine:
            # x:76 y:71
            OperatableStateMachine.add('Prepare_UAV',
                                        self.use_behavior(Prepare_UAVSM, 'Prepare_UAV'),
                                        transitions={'finished': 'GotoAltitude', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'simulation': 'simulation'})

            # x:326 y:71
            OperatableStateMachine.add('GotoAltitude',
                                        self.use_behavior(GotoAltitudeSM, 'GotoAltitude'),
                                        transitions={'reached': 'Goto_Waiting_Spot', 'failed': 'Emergency_Landing'},
                                        autonomy={'reached': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'goal': 'flying_altitude'})

            # x:971 y:268
            OperatableStateMachine.add('Flythrough_Window_Inside',
                                        WindowFlyerActionState(action_server_name=window_flyer_action_server_name, yaw_offset=3.14),
                                        transitions={'successed': 'Fire_Detect_Set_Realsense_Mode', 'window_not_found': 'Emergency_Landing', 'already_flythrough': 'Emergency_Landing', 'server_not_initialized': 'Emergency_Landing', 'error': 'Emergency_Landing', 'preempted': 'Emergency_Landing'},
                                        autonomy={'successed': Autonomy.Off, 'window_not_found': Autonomy.Off, 'already_flythrough': Autonomy.Off, 'server_not_initialized': Autonomy.Off, 'error': Autonomy.Off, 'preempted': Autonomy.Off},
                                        remapping={'window_id': 'window_id', 'window_position': 'window_position'})

            # x:774 y:74
            OperatableStateMachine.add('Reset_Window_Estimation',
                                        ServiceTriggerState(service_topic=window_position_estimation_reset_topic, state_name="WindowEstimation"),
                                        transitions={'finished': 'Wait', 'failed': 'Emergency_Landing'},
                                        autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

            # x:965 y:178
            OperatableStateMachine.add('Wait_For_Window_Estimation',
                                        WaitForMsgState(topic=window_position_estimation_closest_window_topic, wait_time=5, function=lambda x: True, output_keys=['window_id'], output_function=self.window_detection_cb),
                                        transitions={'successed': 'Flythrough_Window_Inside', 'failed': 'Call_Land_Home'},
                                        autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'window_id': 'window_id'})

            # x:1008 y:74
            OperatableStateMachine.add('Wait',
                                        WaitState(wait_time=wait_time_after_flying),
                                        transitions={'done': 'Wait_For_Window_Estimation'},
                                        autonomy={'done': Autonomy.Off})

            # x:412 y:579
            OperatableStateMachine.add('Call_Land_Home',
                                        ServiceTriggerState(service_topic=land_home_service_topic, state_name=None),
                                        transitions={'finished': 'finished', 'failed': 'Call_Land_Home'},
                                        autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

            # x:519 y:329
            OperatableStateMachine.add('Emergency_Landing',
                                        ServiceTriggerState(service_topic=e_land_topic, state_name="ControlManager"),
                                        transitions={'finished': 'failed', 'failed': 'Emergency_Landing'},
                                        autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

            # x:555 y:69
            OperatableStateMachine.add('Goto_Waiting_Spot',
                                        self.use_behavior(goto_referenceSM, 'Goto_Waiting_Spot'),
                                        transitions={'reached': 'Reset_Window_Estimation', 'failed': 'Emergency_Landing'},
                                        autonomy={'reached': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'goal': 'waiting_spot'})

            # x:563 y:476
            OperatableStateMachine.add('Fire_Detect_Set_Realsense_Mode',
                                        ServiceSetBoolState(service_topic=fire_detect_set_realsense_mode_topic, request=True, state_name=None),
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    @staticmethod
    def window_detection_cb(message, userdata):
        userdata.window_id = message.objects[0].id

    # [/MANUAL_FUNC]
