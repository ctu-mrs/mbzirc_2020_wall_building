# ids of internal state
uint8 IDLE       = 0
uint8 PREPARING  = 1
uint8 FLYING     = 2
uint8 GRASPING   = 3
uint8 PLACING    = 4
uint8 WAITING    = 5

uint8 TYPE_COUNT = 6       # number of possible states

uint8 state_id             # id of internal state machine (see predefine values above) 
