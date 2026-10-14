bool goal1_received
bool goal1_achieved
bool goal2_received
bool goal2_achieved
bool emergency          # 0 is no emergency, 1 is handling emergency
bool human_found        # 0 is not found, 1 means found
geometry_msgs/Point human_location  # (x,y,z) location of the human found in the /map frame - Ignore z value
