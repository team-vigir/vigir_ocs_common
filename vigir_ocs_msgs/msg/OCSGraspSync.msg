#Typically only sync one grasp item at one time, using enums as a way to ignore changing other items
bool show_grasp

bool left_hand_lock
bool right_hand_lock
int16 template_id

int16 grasp_id

uint8 SHOW_GRASP = 1
uint8 HAND_LOCKS = 2
uint8 GRASP_ID   = 3
uint8 SYNC_ALL   = 4

uint8 sync_mode

#extra data, necessary for multiple sync modes
string hand
string host


