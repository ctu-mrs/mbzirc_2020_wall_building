#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from actionlib_msgs.msg import GoalStatus

# example import of required action
from wall_following.msg import FollowingAction, FollowingGoal

class WallFollowingActionState(EventState):
    '''
    State for calling wall_following action server.
    '''

    def __init__(self, action_server_name, desired_yaw, desired_speed, desired_wall_distance):
        # See example_state.py for basic explanations.
        super(WallFollowingActionState, self).__init__( outcomes = ['successed', 'aborted', 'preempted'])

        self._topic = action_server_name
        self._desired_yaw = desired_yaw
        self._desired_speed = desired_speed
        self._desired_wall_distance = desired_wall_distance
        self._client = ProxyActionClient({self._topic: FollowingAction}) # pass required clients as dict (topic: type)
        Logger.loginfo('[WallFollowing]: Action client initialized')
        # It may happen that the action client fails to send the action goal.
        self._error = False


    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            return 'aborted'

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            status = self._client.get_state(self._topic)
            result = self._client.get_result(self._topic)
            if status == GoalStatus.SUCCEEDED:
                Logger.loginfo("[WallFollowing]: %s" % str(result.message))
                return 'successed'
            elif status == GoalStatus.PREEMPTED:
                Logger.logwarn('[WallFollowing]: Action preempted')
                return 'preempted'
            elif status in [GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logerr('[WallFollowing]: Action failed: %s' % str(result.message))
                return 'aborted'

        """feedback params are"""
        feedback = self._client.get_feedback(self._topic)
        if feedback is not None:
            Logger.loginfo("[WallFollowing]: Current feedback msg: %s" % feedback.message)

    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # Create the goal.
        goal = FollowingGoal()

        #fill the goal
        goal.yaw = self._desired_yaw
        goal.wall_distance = self._desired_wall_distance
        goal.speed = self._desired_speed

        # Send the goal.
        self._error = False # make sure to reset the error state since a previous state execution might have failed
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logerr('[WallFollowing]: Failed to send the goal:\n%s' % str(e))
            self._error = True


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('[WallFollowing]: Cancelled active action goal.')

