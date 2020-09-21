#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from actionlib_msgs.msg import GoalStatus
import mrs_msgs.srv
import copy

# example import of required action
from lidar_flier.msg import lfAction, lfGoal

class LidarFlierActionState(EventState):
    '''
    State for calling lidar_flier action server.
    '''

    def result_to_outcome(self, argument):
        switcher = {
            0: "successed",                 # 0 - orbit completed
            1: "successed",                 # 1 - goto completed
            2: "server_not_initialized",    # 2 - not initialized
            3: "no_valid_points_in_scan",   # 3 - no valid points in laser scan (too far from building)
            4: "stop_requested",            # 4 - stop requested
        }
        return switcher.get(argument, "error")

    def __init__(self, action_server_name):
        # See example_state.py for basic explanations.
        super(LidarFlierActionState, self).__init__( outcomes = ['successed', 'no_valid_points_in_scan', 'stop_requested', 'server_not_initialized', 'error', 'preempted'],
                                                     input_keys = ['goal'])

        self._topic = action_server_name
        self._client = ProxyActionClient({self._topic: lfAction}) # pass required clients as dict (topic: type)
        Logger.loginfo('[LidarFlierAction]: Action client initialized')
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
            Logger.loginfo("[LidarFlierAction]: message %s" % str(result.message))
            if status == GoalStatus.SUCCEEDED:
                Logger.loginfo("[LidarFlierAction]: %s" % str(result.message))
                return self.result_to_outcome(result.result_id)

            elif status == GoalStatus.PREEMPTED:
                Logger.logwarn('[LidarFlierAction]: Action preempted')
                return 'preempted'

            elif status in [GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logerr('[LidarFlierAction]: Action failed: %s' % str(result.message))
                return self.result_to_outcome(result.result_id)


        """feedback params are"""
        feedback = self._client.get_feedback(self._topic)
        if feedback is not None:
            Logger.loginfo("[LidarFlierAction]: Feedback msg: %3d" % feedback.progress)

    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # Create the goal.
        goal = copy.deepcopy(userdata.goal)
        Logger.loginfo('[LidarFlierAction]: Action client received new goal:')
        Logger.loginfo(goal)

        # Send the goal.
        self._error = False # make sure to reset the error state since a previous state execution might have failed
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logerr('[LidarFlierAction]: Failed to send the goal:\n%s' % str(e))
            self._error = True


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('[LidarFlierAction]: Cancelled active action goal.')

