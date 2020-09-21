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
from mrs_flexbe_behaviors.goto_sm import GotoSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Sep 17 2019
@author: Vojtech Spurny
'''
class Start_and_flySM(Behavior):
    '''
    neco
    '''


    def __init__(self):
        super(Start_and_flySM, self).__init__()
        self.name = 'Start_and_fly'

        # parameters of this behavior

        # references to used behaviors
        self.add_behavior(Prepare_UAVSM, 'Prepare_UAV')
        self.add_behavior(GotoSM, 'Goto')

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:836 y:81, x:130 y:525
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:173 y:71
            OperatableStateMachine.add('Prepare_UAV',
                                        self.use_behavior(Prepare_UAVSM, 'Prepare_UAV',
                                            default_keys=['simulation']),
                                        transitions={'finished': 'Goto', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

            # x:411 y:71
            OperatableStateMachine.add('Goto',
                                        self.use_behavior(GotoSM, 'Goto',
                                            default_keys=['goal']),
                                        transitions={'reached': 'finished', 'failed': 'failed'},
                                        autonomy={'reached': Autonomy.Inherit, 'failed': Autonomy.Inherit})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
