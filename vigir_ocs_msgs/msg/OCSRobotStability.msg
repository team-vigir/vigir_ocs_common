# Contains basic stability information projected into the world frame
Header                 header
# Stability metric related to ratio of distance of CoP(ZMP) to boundary of support polygon to the sqrt of (foot area) 0=falling, 255=best single support
uint8                  stability
# Stance state
uint8                  stance
# Support mode -200.0 unknown, 100.0 left, 700.0 right, 400,0 double support (for plotting in range of forces)
std_msgs/Float32       support_mode
# Location of pelvis in world frame
geometry_msgs/Pose     pelvis_pose
# Center of mass relative to pelvis in world gravitational frame
geometry_msgs/Point32  CoM_gravity
# Gravity vector in pelvis frame
geometry_msgs/Point32  gravity_pelvis
# Weighted center of pressure relative to pelvis
geometry_msgs/Point32  CoP
# ZMP should be coincident with CoP
geometry_msgs/Point32  ZMP
# Support polygon relative to pelvis
geometry_msgs/Polygon  support_polygon

# These are additional temporary for debugging will remove to save bandwidth
# Center of mass relative to pelvis frame
geometry_msgs/Point32  CoM_pelvis
# Weighted center of pressure relative to pelvis
geometry_msgs/Point32  CoP_posn_left
geometry_msgs/Point32  CoP_posn_right
geometry_msgs/Point32  CoP_force_left
geometry_msgs/Point32  CoP_force_right

