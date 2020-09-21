time stamp
string robot_name                      
int16 current_brick_id                  # id of a brick that the drone wants to place
PlanStateTypes state                    # structure with id explaining internal state machine
int16[] placed_brick_ids                # ids of already placed bricks
float64 remaining_time_to_finish_task   # estimated time till placing brick
