#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
#from chores.msg import DoDishesAction, DoDishesGoal
from mbzirc_msgs.msg import GraspingAction, GraspingActionGoal, GraspingGoal
from geometry_msgs.msg import Point
import rospy


class BrickGraspingActionState(EventState):

    """ brick grasping results """
    RESULT_SUCCESS             = 0
    RESULT_STOPPED_BY_COMMAND  = 1
    RESULT_OBJECT_NOT_RELIABLE = 2
    RESULT_OBJECT_NOT_VISIBLE  = 3
    RESULT_GRASPING_TIMOUT     = 4
    RESULT_GRASPING_FAILED     = 5

    """ brick grasping object types """
    OBJECT_RED  = 1
    OBJECT_GREEN  = 2
    OBJECT_BLUE    = 3
    OBJECT_ORANGE = 4
    WALL = 5

    """ result ids """
    RESULT_SUCCESS = 0
    RESULT_STOPPED_BY_COMMAND = 1
    RESULT_OBJECT_NOT_RELIABLE = 2
    RESULT_OBJECT_NOT_VISIBLE = 3
    RESULT_GRASPING_TIMOUT = 4
    RESULT_GRASPING_FAILED = 5

    def __init__(self, return_altitude):
        # See example_state.py for basic explanations.
        super(BrickGraspingActionState, self).__init__( outcomes = ['grasped', 'grasping_error'],
                                                       input_keys = ['brick_type'])


        self._topic = 'brick_grasping'
        self._client = ProxyActionClient({self._topic: GraspingAction}) # pass required clients as dict (topic: type)
        self._return_altitude = return_altitude
        Logger.loginfo('[GraspingAction]: Client initialized')
        # It may happen that the action client fails to send the action goal.
        self._error = False


    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            return 'grasping_error'

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            Logger.loginfo("[GraspingAction]: ended %s" % str(result.message))
            # In this example, we also provide the amount of cleaned dishes as output key.
            if result.success:
                return 'grasped'
            else:
                return 'grasping_error'

        """feedback params are"""
        feedback = self._client.get_feedback(self._topic)
        if feedback is not None:
            rospy.loginfo_throttle(5,"[GraspingAction]: Current feedback msg: %s" % str(feedback.message))


    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        brick_type = userdata.brick_type

        # Create the goal.
        goal = GraspingGoal()

        #fill the goal
        Logger.loginfo("[GraspingAction]: want to grasp brick type %s" % (str(brick_type)))
        goal.goal = brick_type
        goal.return_altitude = self._return_altitude #where to return after grasping

        # Send the goal.
        self._error = False # make sure to reset the error state since a previous state execution might have failed
        try:
            Logger.loginfo("[GraspingAction]: sending goal %s to topic %s" % (str(goal),str(self._topic)))
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn('[GraspingAction]: Failed to send the GraspingAction command:\n%s' % str(e))
            self._error = True


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('[GraspingAction]: Cancelled active action goal.')

