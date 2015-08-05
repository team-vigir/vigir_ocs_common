string host

# object type
# defines which type of object has been selected for manipulation
uint8 type

# object type values
uint8 END_EFFECTOR            =     0
uint8 TEMPLATE                =     1
uint8 FOOTSTEP                =     2
uint8 FOOTSTEP_GOAL           =     3
uint8 NOTHING                 =     4

# object id
# what is the id of the object selected among those of the same type
uint32 id

# for the end-effectors, we use special defines for each
int8  UNLOCK_ARMS             =    -1
uint8 LEFT_ARM                =     0
uint8 RIGHT_ARM               =     1
uint8 PELVIS                  =     2

string operator_type
