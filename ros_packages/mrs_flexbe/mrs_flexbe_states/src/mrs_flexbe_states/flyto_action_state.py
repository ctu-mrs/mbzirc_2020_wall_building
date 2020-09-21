#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
from mbzirc_flyto_planner.msg import FlyToAction, FlyToGoal
from geometry_msgs.msg import Point
from actionlib_msgs.msg import GoalStatus



class FlyToActionState(EventState):
    '''
    State for calling mbzirc_flyto_planner action server.
    '''

    def __init__(self, action_server_name):
        # See example_state.py for basic explanations.
        super(FlyToActionState, self).__init__( outcomes = ['reached', 'aborted' , 'preempted'],
                                               input_keys = ['goal', 'flying_velocity'])

        self._topic = action_server_name
        self._client = ProxyActionClient({self._topic: FlyToAction}) # pass required clients as dict (topic: type)
        Logger.loginfo('[FlyToAction]: Action client initialized')
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
            #print("result type",type(result))
            #print("status type",type(status))
            if status == GoalStatus.SUCCEEDED:
                Logger.loginfo("[FlyToAction]: %s" % str(result.message))
                Logger.loginfo("[FlyToAction]: on exit goal reached x,y,z,yaw is %s,%s,%s,%s" % (str(userdata.goal.x),str(userdata.goal.y),str(userdata.goal.z),str(userdata.goal.yaw)))
                return 'reached'
            elif status == GoalStatus.PREEMPTED:
                Logger.logwarn('[FlyToAction]: Action preempted')
                return 'preempted'
            elif status in [GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logerr('[FlyToAction]: Action failed: %s' % str(result.message))
                return 'aborted'

        """feedback params are"""
        feedback = self._client.get_feedback(self._topic)
        if feedback is not None:
            Logger.loginfo("[FlyToAction]: Current feedback msg: distance_to_goal: %f" % feedback.distance_to_goal)


    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # Create the goal.
        goal = FlyToGoal()

        Logger.loginfo("[FlyToAction]: on enter goal x,y,z,yaw is %s,%s,%s,%s" % (str(userdata.goal.x),str(userdata.goal.y),str(userdata.goal.z),str(userdata.goal.yaw)))
        
        #fill the goal
        goal.points.append(userdata.goal)
        goal.velocity = userdata.flying_velocity

        # Send the goal.
        self._error = False # make sure to reset the error state since a previous state execution might have failed
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn('[FlyToAction]: Failed to send the FlyToAction goal:\n%s' % str(e))
            self._error = True


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('[FlyToAction]: Cancelled active action goal.')

