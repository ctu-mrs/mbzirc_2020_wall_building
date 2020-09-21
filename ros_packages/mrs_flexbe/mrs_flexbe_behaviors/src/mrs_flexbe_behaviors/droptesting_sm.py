#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mrs_flexbe_states.brick_placing_action_state import BrickPlacingActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from mrs_msgs.msg import TrackerPoint
# [/MANUAL_IMPORT]


'''
Created on Wed Sep 25 2019
@author: Robert Penicka
'''
class DropTestingSM(Behavior):
	'''
	Testing behavior for dropping brick
	'''


	def __init__(self):
		super(DropTestingSM, self).__init__()
		self.name = 'DropTesting'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['placing_position'])
		_state_machine.userdata.placing_position = TrackerPoint(0,0,0.3,0)

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:161 y:131
			OperatableStateMachine.add('BrickPlacing',
										BrickPlacingActionState(),
										transitions={'placed': 'finished', 'placing_error': 'failed'},
										autonomy={'placed': Autonomy.Off, 'placing_error': Autonomy.Off},
										remapping={'placing_position': 'placing_position', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
