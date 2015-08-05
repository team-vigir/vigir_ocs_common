uint8 FEEDBACK  = 0  # from manager to nodes -> latest poses
uint8 CLEAR     = 1  # from nodes to manager -> if the plan request was succesful, need to clear goal
uint8 GOAL      = 2  # from nodes to manager -> update goal
uint8 LEFT      = 3  # from nodes to manager -> update left foot
uint8 RIGHT     = 4  # from nodes to manager -> update right foot
uint8 UPDATE    = 5  # from nodes to manager -> time to use planner to update feet (mouse release)

uint8 mode

# goal_pose coming from nav goal tool
geometry_msgs/PoseStamped goal_pose

# goal pose for each foot
geometry_msgs/PoseStamped left_foot
geometry_msgs/PoseStamped right_foot
