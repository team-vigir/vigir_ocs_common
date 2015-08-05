uint8 SYNCED                 =  0  # Everything is fine, can execute
uint8 EMPTY_PLAN_LIST        =  1  # Haven't created a plan yet
uint8 EMPTY_PLAN             =  2  # Plan is invalid (0 steps)
uint8 NEED_PLAN_VALIDATION   =  3  # Need to validate plan onboard
uint8 PROCESSING_OCS_PLAN    =  4  # In the process of updating the current plan using the OCS planner

uint8 status

# tells the ocs what is going to be sent over the comms bridge when validating
uint8 EDITED_STEPS =  1
uint8 CURRENT_PLAN =  2
uint8 GOAL_FEET    =  3
uint8 GOAL         =  4

uint8 validate_mode
