#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger


class SetVariableToTrueState(EventState):
	'''
        Sets the variable to True

	-- input_keys 	
	-- output_keys 	
        -- function

	<= done 				Example for a failure outcome.

	'''

	def __init__(self, key, function):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(SetVariableToTrueState, self).__init__(outcomes = ['done'], input_keys = key, output_keys = key)
                self._key = key
                self._function = function

	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
                self._function(userdata)
                Logger.loginfo('[SetVariableToTrue]: %s == True' % str(self._key))
                return 'done'
		
