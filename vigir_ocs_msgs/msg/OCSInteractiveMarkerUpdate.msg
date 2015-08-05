string client_id    # node_name so we know who sent this update request

uint8 FEEDBACK = 0  # it is based on feedback, so will not generate events
uint8 SET_POSE = 1  # will generate events because it's setting the position
uint8 update_mode

string topic
geometry_msgs/PoseStamped pose

uint8 event_type    # types defined in visualization_msgs::InteractiveMarkerFeedback
