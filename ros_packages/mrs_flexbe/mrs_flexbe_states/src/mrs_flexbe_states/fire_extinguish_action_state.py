#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from actionlib_msgs.msg import GoalStatus
import mrs_msgs.srv
import copy

# example import of required action
from fire_detect.msg import firemanAction, firemanGoal

class FireExtinguishActionState(EventState):
    '''
    State for calling fire_detect action server.
    '''

    def result_to_outcome(self, argument):
        switcher = {
            0: "successed",                
            1: "lost_target",  
        }
        return switcher.get(argument, "error")

    def __init__(self, action_server_name):
        # See example_state.py for basic explanations.
        super(FireExtinguishActionState, self).__init__( outcomes = ['successed', 'lost_target', 'error', 'preempted'],
                                                     input_keys = ['goal'], output_keys = ['is_tank_depleted'])

        self._topic = action_server_name
        self._client = ProxyActionClient({self._topic: firemanAction}) # pass required clients as dict (topic: type)
        Logger.loginfo('[FireExtinguish]: Action client initialized')
        # It may happen that the action client fails to send the action goal.
        self._error = False


    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            userdata.is_tank_depleted = False
            return 'error'

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            status = self._client.get_state(self._topic)
            result = self._client.get_result(self._topic)
            Logger.loginfo("[FireExtinguish]: message %s" % str(result.message))
            if status == GoalStatus.SUCCEEDED:
                Logger.loginfo("[FireExtinguish]: %s" % str(result.message))
                userdata.is_tank_depleted = True
                return self.result_to_outcome(result.result_id)

            elif status == GoalStatus.PREEMPTED:
                userdata.is_tank_depleted = False
                Logger.logwarn('[FireExtinguish]: Action preempted')
                return 'preempted'

            elif status in [GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logerr('[FireExtinguish]: Action failed: %s' % str(result.message))
                userdata.is_tank_depleted = False
                return self.result_to_outcome(result.result_id)


        """feedback params are"""
        feedback = self._client.get_feedback(self._topic)
        if feedback is not None:
            Logger.loginfo("[FireExtinguish]: Feedback - Water reservoir %3d %%" % feedback.water_percent)

    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # Create the goal.
        goal = firemanGoal()
        goal.where_x = userdata.goal.x
        goal.where_y = userdata.goal.y
        goal.where_z = userdata.goal.z
        goal.extinguish = True

        Logger.loginfo('[FireExtinguish]: Action client received new goal:')
        Logger.loginfo(goal)

        # Send the goal.
        self._error = False # make sure to reset the error state since a previous state execution might have failed
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logerr('[FireExtinguish]: Failed to send the goal:\n%s' % str(e))
            self._error = True


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('[FireExtinguish]: Cancelled active action goal.')

