#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mrs_flexbe_states.brick_grasping_action_state import BrickGraspingActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Sep 24 2019
@author: Robert Penicka
'''
class GraspTestingSM(Behavior):
	'''
	Behavior for testing graping action server state
	'''


	def __init__(self):
		super(GraspTestingSM, self).__init__()
		self.name = 'GraspTesting'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:213 y:312
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['goal_brick'])
		_state_machine.userdata.goal_brick = 1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('grasp',
										BrickGraspingActionState(),
										transitions={'grasped': 'finished', 'grasping_error': 'failed'},
										autonomy={'grasped': Autonomy.Off, 'grasping_error': Autonomy.Off},
										remapping={'goal_brick': 'goal_brick', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
