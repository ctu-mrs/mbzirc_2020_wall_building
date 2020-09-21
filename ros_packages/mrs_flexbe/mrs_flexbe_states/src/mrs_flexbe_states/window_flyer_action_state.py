#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from actionlib_msgs.msg import GoalStatus
import mrs_msgs.srv

# example import of required action
from window_flyer.msg import FlythroughWindowAction, FlythroughWindowGoal

class WindowFlyerActionState(EventState):
    '''
    State for calling window_flyer action server.
    '''

    def result_to_outcome(self, argument):
        switcher = {
            0: "successed",
            1: "server_not_initialized",
            2: "window_not_found",
            3: "already_flythrough",
            4: "timeout",
        }
        return switcher.get(argument, "error")

    def __init__(self, action_server_name, just_approach, yaw_offset = 0):
        # See example_state.py for basic explanations.
        super(WindowFlyerActionState, self).__init__( outcomes = ['successed', 'window_not_found', 'already_flythrough', 'server_not_initialized', 'error', 'timeout', 'preempted'],
                                                     input_keys = ['window_id'], output_keys = ['window_position'])

        self._topic = action_server_name
        self._yaw_offset = yaw_offset
        self._just_approach = just_approach
        self._client = ProxyActionClient({self._topic: FlythroughWindowAction}) # pass required clients as dict (topic: type)
        Logger.loginfo('[WindowFlyerAction]: Action client initialized')
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
                Logger.loginfo("[WindowFlyerAction]: %s" % str(result.message))
                Logger.loginfo("[WindowFlyerAction]: Finished on position: %s" % str(result.final_position))

                reference_srv = mrs_msgs.srv.ReferenceStampedSrvRequest()
                reference_srv.header = result.final_position.header
                reference_srv.reference = result.final_position.reference
                reference_srv.reference.yaw = reference_srv.reference.yaw + self._yaw_offset
                userdata.window_position = reference_srv
                return 'successed'

            elif status == GoalStatus.PREEMPTED:
                Logger.logwarn('[WindowFlyerAction]: Action preempted')
                return 'preempted'

            elif status in [GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logerr('[WindowFlyerAction]: Action failed: %s' % str(result.message))
                return self.result_to_outcome(result.result_id)


        """feedback params are"""
        feedback = self._client.get_feedback(self._topic)
        if feedback is not None:
            Logger.loginfo("[WindowFlyerAction]: Feedback msg: %s" % feedback.message)

    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # Create the goal.
        goal = FlythroughWindowGoal()

        #fill the goal
        goal.window_id = userdata.window_id
        goal.fly_inside_confirmation = not self._just_approach

        # Send the goal.
        self._error = False # make sure to reset the error state since a previous state execution might have failed
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logerr('[WindowFlyerAction]: Failed to send the goal:\n%s' % str(e))
            self._error = True


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('[WindowFlyerAction]: Cancelled active action goal.')

