#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mrs_flexbe_states.service_get_ctop_plan import ServiceGetCtopPlan
from mrs_flexbe_states.service_set_plan_into_plan_keeper import ServiceSetPlanIntoPlanKeeper
from mrs_flexbe_states.service_next_brick_in_plan import ServiceGetNextBrickInPlan
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Sep 25 2019
@author: VS
'''
class plan_testingSM(Behavior):
	'''
	testing ctop_planner and plan_keeper
	'''


	def __init__(self):
		super(plan_testingSM, self).__init__()
		self.name = 'plan_testing'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		ctop_plan_service_topic = "ctop_planner/plan"
		plan_keeper_plan_in_topic = "plan_keeper/set_complete_plan"
		plan_keeper_get_brick_plan_topic = "plan_keeper/get_brick_plan"
		# x:961 y:150, x:553 y:239
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.num_robots = 3
		_state_machine.userdata.remaining_time_s = 500
		_state_machine.userdata.max_planning_time_s = 30
		_state_machine.userdata.times_finish_actual_s = [0, 0, 0]
		_state_machine.userdata.builded_brick_ids = []
		_state_machine.userdata.robot_name = "uav1"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:92 y:74
			OperatableStateMachine.add('Get_CTOP_Plan',
										ServiceGetCtopPlan(service_topic=ctop_plan_service_topic),
										transitions={'successed': 'Set_Plan_Into_Plan_Keeper', 'failed': 'Get_CTOP_Plan'},
										autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'num_robots': 'num_robots', 'remaining_time_s': 'remaining_time_s', 'max_planning_time_s': 'max_planning_time_s', 'times_finish_actual_s': 'times_finish_actual_s', 'builded_brick_ids': 'builded_brick_ids', 'plans': 'plans'})

			# x:319 y:74
			OperatableStateMachine.add('Set_Plan_Into_Plan_Keeper',
										ServiceSetPlanIntoPlanKeeper(service_topic=plan_keeper_plan_in_topic),
										transitions={'successed': 'Get_Next_Brick_Plan', 'failed': 'failed'},
										autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'plans': 'plans'})

			# x:627 y:74
			OperatableStateMachine.add('Get_Next_Brick_Plan',
										ServiceGetNextBrickInPlan(service_topic=plan_keeper_get_brick_plan_topic),
										transitions={'successed': 'finished', 'failed': 'failed'},
										autonomy={'successed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_name': 'robot_name', 'plan_for_next_brick': 'plan_for_next_brick'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
