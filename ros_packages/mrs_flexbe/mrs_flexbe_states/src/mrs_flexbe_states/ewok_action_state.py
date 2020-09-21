#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from actionlib_msgs.msg import GoalStatus
import mrs_msgs.srv
import copy

# example import of required action
from mrs_ewok.msg import ewokAction, ewokGoal

class EwokActionState(EventState):
    '''
    State for calling window_flyer action server.
    '''

    def result_to_outcome(self, argument):
        switcher = {
            0: "successed",
        }
        return switcher.get(argument, "error")

    def __init__(self, action_server_name):
        # See example_state.py for basic explanations.
        super(EwokActionState, self).__init__( outcomes = ['successed', 'error','preempted'],
                                                     input_keys = ['goal'])

        self._topic = action_server_name
        self._client = ProxyActionClient({self._topic: ewokAction}) # pass required clients as dict (topic: type)
        Logger.loginfo('[EwokAction]: Action client initialized')
        # It may happen that the action client fails to send the action goal.
        self._error = False


    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            return 'error'

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            status = self._client.get_state(self._topic)
            result = self._client.get_result(self._topic)
            if status == GoalStatus.SUCCEEDED:
                Logger.loginfo("[EwokAction]: %s" % str(result.message))
                Logger.loginfo("[EwokAction]: Finished on position: %s" % str(result.final_position))

                return 'successed'

            elif status == GoalStatus.PREEMPTED:
                Logger.logwarn('[EwokAction]: Action preempted')
                return 'preempted'

            elif status in [GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logerr('[EwokAction]: Action failed: %s' % str(result.message))
                return self.result_to_outcome(result.result_id)


        """feedback params are"""
        feedback = self._client.get_feedback(self._topic)
        if feedback is not None:
            Logger.loginfo("[EwokAction]: Feedback msg: %s" % feedback.message)

    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        #fill the goal
        goal = copy.deepcopy(userdata.goal)

        # Send the goal.
        self._error = False # make sure to reset the error state since a previous state execution might have failed
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logerr('[EwokAction]: Failed to send the goal:\n%s' % str(e))
            self._error = True


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('[EwokAction]: Cancelled active action goal.')

