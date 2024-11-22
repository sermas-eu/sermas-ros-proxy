import os

ROS_USERS_LANDMARKS_TOPIC = os.environ.get(
    "ROS_USERS_LANDMARKS_TOPIC", "/face_landmarks_node/users_landmarks")
ROS_MUTUAL_GAZE_TOPIC = os.environ.get(
    "ROS_MUTUAL_GAZE_TOPIC", "/mutual_gaze_output")
ROS_SESSION_TOPIC = os.environ.get(
    "ROS_SESSION_TOPIC", "/interaction_manager_node/session_event")
ROS_USER_POSITION_TOPIC = os.environ.get(
    "ROS_USER_POSITION_TOPIC", "/user/position")
ROS_ODOMETRY_TOPIC = os.environ.get("ROS_ODOMETRY_TOPIC", "/odom")
ROS_GOALPOSE_TOPIC = os.environ.get(
    "ROS_GOALPOSE_TOPIC", "/move_base_simple/goal")
ROS_INITIALPOSE_TOPIC = os.environ.get("ROS_INITIALPOSE_TOPIC", "/initialpose")
ROS_VELOCITY_TOPIC = os.environ.get("ROS_VELOCITY_TOPIC", "/cmd_vel")
ROS_VIDEO_TOPIC = os.environ.get(
    "ROS_VIDEO_TOPIC", "/v4l/camera/image_raw/compressed")
ROS_ACTUATE_TOPIC = os.environ.get(
    "ROS_ACTUATE_TOPIC", "/actuate")
ROS_ARM_STATE_TOPIC = os.environ.get(
    "ROS_ARM_STATE_TOPIC", "/arm_controller/state")
ROS_ARM_TRAJECTORY_TOPIC = os.environ.get(
    "ROS_ARM_TRAJECTORY_TOPIC", "/arm_controller/joint_trajectory")
ROS_GRIPPER_STATE_TOPIC = os.environ.get(
    "ROS_GRIPPER_STATE_TOPIC", "/grip_controller/state")
ROS_GRIPPER_TRAJECTORY_TOPIC = os.environ.get(
    "ROS_GRIPPER_TRAJECTORY_TOPIC", "/grip_controller/joint_trajectory")

SERMAS_USER_DETECTION_TOPIC = os.environ.get(
    "SERMAS_USER_DETECTION_TOPIC", "detection/user")
SERMAS_INTENT_DETECTION_TOPIC = os.environ.get(
    "SERMAS_INTENT_DETECTION_TOPIC", "detection/interaction")
SERMAS_ROBOT_STATUS_TOPIC = os.environ.get(
    "SERMAS_ROBOT_STATUS_TOPIC", "robotics/status")
SERMAS_ROBOTCMD_TOPIC = os.environ.get(
    "SERMAS_ROBOTCMD_TOPIC", "robotics/move")
SERMAS_ROBOTINITIALPOSE_TOPIC = os.environ.get(
    "SERMAS_ROBOTINITIALPOSE_TOPIC", "robotics/initialpose")
SERMAS_ROBOT_VIDEO_FEED_TOPIC = os.environ.get(
    "SERMAS_ROBOT_VIDEO_FEED_TOPIC", "robotics/videofeed")
SERMAS_ROBOT_ACTUATE_TOPIC = os.environ.get(
    "SERMAS_ROBOT_ACTUATE_TOPIC", "robotics/actuate")
SERMAS_ROBOT_OP_STATE_TOPIC = os.environ.get(
    "SERMAS_ROBOT_OP_STATE_TOPIC", "robotics/opstate")
SERMAS_AGENT_CHANGED_TOPIC = os.environ.get(
    "SERMAS_ROBOT_OP_STATE_TOPIC", "session/agent/changed/+")
