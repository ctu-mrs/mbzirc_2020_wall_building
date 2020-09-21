#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from mbzirc_msgs.msg import GraspingAction, GraspingActionGoal, GraspingGoal
from geometry_msgs.msg import Point
import rospy

class BrickPlacingActionState(EventState):

    """ brick grasping results """
    RESULT_SUCCESS = 0
    RESULT_STOPPED_BY_COMMAND = 1
    RESULT_OBJECT_NOT_RELIABLE = 2
    RESULT_OBJECT_NOT_VISIBLE = 3
    RESULT_GRASPING_TIMOUT = 4
    RESULT_GRASPING_FAILED = 5

    """ brick grasping object types """
    
    OBJECT_RED = 1
    OBJECT_GREEN = 2
    OBJECT_BLUE = 3
    OBJECT_ORANGE = 4
    WALL = 5
    WALL_RED = 6
    WALL_GREEN = 7
    WALL_BLUE = 8

    BRCIK_TYPE_TO_WALL_PLACING_TYPE = {OBJECT_RED:WALL_RED, OBJECT_GREEN:WALL_GREEN, OBJECT_BLUE:WALL_BLUE}

    """ result ids """
    RESULT_SUCCESS = 0
    RESULT_STOPPED_BY_COMMAND = 1
    RESULT_OBJECT_NOT_RELIABLE = 2
    RESULT_OBJECT_NOT_VISIBLE = 3
    RESULT_GRASPING_TIMOUT = 4
    RESULT_GRASPING_FAILED = 5

    def __init__(self):
        # See example_state.py for basic explanations.
        super(BrickPlacingActionState, self).__init__(outcomes=['placed', 'placing_error'],
                                                      input_keys=['placing_position','brick_type','return_altitude'],
                                                      output_keys=['placing_result'])

        self._topic = 'brick_grasping'
        self._client = ProxyActionClient({self._topic: GraspingAction})  # pass required clients as dict (topic: type)
        
        Logger.loginfo('[PlacingAction]: Client initialized')
        # It may happen that the action client fails to send the action goal.
        self._error = False

    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            Logger.logerr('[PlacingAction]: ending with "placed"')
            return 'placing_error'

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            Logger.loginfo("[PlacingAction]: ended with result %s" % str(result.message))

            if result.success:
                Logger.loginfo('[PlacingAction]: ending with "placed"')
                userdata.placing_result = True
                return 'placed'
            else:
                Logger.logwarn('[PlacingAction]: ending with "placing_error"')
                userdata.placing_result = False
                return 'placing_error'

        """feedback params are"""
        feedback = self._client.get_feedback(self._topic)
        if feedback is not None:
            rospy.loginfo_throttle(5,"[PlacingAction]: Current feedback msg: %s" % (str(feedback.message)))

    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        placing_position = userdata.placing_position
        brick_type = userdata.brick_type
        return_altitude = userdata.return_altitude
        # Create the goal.
        goal = GraspingGoal()

        if BrickPlacingActionState.BRCIK_TYPE_TO_WALL_PLACING_TYPE.get(brick_type) is None:
            Logger.logerr('[PlacingAction]: No placing action for brick type %s' % str(brick_type))
            goal.goal = self.WALL
        else:
            goal.goal = BrickPlacingActionState.BRCIK_TYPE_TO_WALL_PLACING_TYPE[brick_type]

        goal.placing_position.position.x = placing_position.position.x  # used only for dropping
        goal.placing_position.position.y = placing_position.position.y  # used only for dropping
        goal.placing_position.position.z = placing_position.position.z  # used only for dropping
        goal.placing_position.heading = placing_position.heading  # used only for dropping
        goal.return_altitude = return_altitude

        # Send the goal.
        self._error = False  # make sure to reset the error state since a previous state execution might have failed
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn('[PlacingAction]: Failed to send the GraspingAction command:\n%s' % str(e))
            self._error = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('[PlacingAction]: Cancelled active action goal.')

