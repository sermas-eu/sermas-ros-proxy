# Mutual gaze users data message definition 
std_msgs/Header header

users_landmarks_msgs/MultipleUsersLandmarks users_data

int32[] looking_at_camera # 0 idle, 1 looking, 2 not_looking

int32 sequence_id # sequence id of the message