import logging
import os
import numpy as np
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String

from ros_proxy.integrations.base_class import IntegrationBaseClass
from ros_proxy.config.topics import ROS_ACTUATE_TOPIC, ROS_ARM_STATE_TOPIC, ROS_ARM_TRAJECTORY_TOPIC, ROS_GRIPPER_STATE_TOPIC, ROS_GRIPPER_TRAJECTORY_TOPIC, SERMAS_ROBOT_ACTUATE_TOPIC, SERMAS_ROBOT_OP_STATE_TOPIC

ROBOT_ARM_POSITION_TOLERANCE = float(os.environ.get(
    "ROBOT_ARM_POSITION_TOLERANCE", 0.1))
ROBOT_GRIPPER_POSITION_TOLERANCE = float(os.environ.get(
    "ROBOT_GRIPPER_POSITION_TOLERANCE", 0.05))
ROBOT_STATE_OP_ARM = "move"
ROBOT_STATE_OP_GRIPPER = "take"
ROBOT_STATE_STARTED = "started"
ROBOT_STATE_FINISHED = "finished"
ROBOT_STATE_FAILED = "failed"

"""
Forward Robot actuation command from SERMAS toolkit to ROS
"""


class MyCobotRobotActuate(IntegrationBaseClass):
  def __init__(self, ros_node):
    super().__init__(ros_node, SERMAS_ROBOT_ACTUATE_TOPIC)
    self.publisher = ros_node.create_publisher(
        PoseStamped, ROS_ACTUATE_TOPIC, 10)

  def handle_sermas_message(self, msg):
    logging.info("Send robot actuation cmd (topic: %s): %s" %
                 (ROS_ACTUATE_TOPIC, str(msg.payload.decode())))
    ros_msg = String()
    ros_msg.data = msg.payload.decode()
    self.publisher.publish(ros_msg)


"""
Forward Robot arm state to SERMAS toolkit
"""


class MyCobotRobotArmState(IntegrationBaseClass):
  def __init__(self, ros_node):
    super().__init__(ros_node, SERMAS_ROBOT_OP_STATE_TOPIC)
    ros_node.create_subscription(
        JointTrajectoryControllerState, ROS_ARM_STATE_TOPIC, self.handle_ros_message, 10)
    ros_node.create_subscription(
        JointTrajectory, ROS_ARM_TRAJECTORY_TOPIC, self.handle_arm_trajectory, 10)

    logging.info("[MQTT] Subscribing to ROS topic %s" % ROS_ARM_STATE_TOPIC)
    logging.info("[MQTT] Subscribing to ROS topic %s" %
                 ROS_ARM_TRAJECTORY_TOPIC)
    self.cur_state = None
    self.desired_position = None
    self.actual_position = None

  def handle_sermas_message(self, msg):
    pass

  def publish(self, state):
    if self.cur_state == state:
      return
    self.cur_state = state
    d = {"state": {"op": ROBOT_STATE_OP_ARM, "state": self.cur_state}}
    logging.info("Robot arm state: %s" % str(d))
    self.mqtt_client.publish(self.sermas_topic, d)

  def handle_arm_trajectory(self, msg):
    if len(msg.points) == 0:
      return
    pos = msg.points[len(msg.points) - 1].positions
    logging.debug("Requested arm position: %s" % (str(pos)))
    self.desired_position = pos

  def match_target_position(self, current, target):
    for idx, c in enumerate(current):
      if np.abs(c - target[idx]) > ROBOT_ARM_POSITION_TOLERANCE:
        return False
    return True

  def handle_ros_message(self, msg):
    if self.desired_position is None:
      return
    self.actual_position = msg.actual.positions
    match = self.match_target_position(
        self.desired_position, self.actual_position)
    if match:
      self.publish(ROBOT_STATE_FINISHED)
    else:
      self.publish(ROBOT_STATE_STARTED)


"""
Forward Robot gripper state to SERMAS toolkit
"""


class MyCobotRobotGripperState(IntegrationBaseClass):
  def __init__(self, ros_node):
    super().__init__(ros_node,  SERMAS_ROBOT_OP_STATE_TOPIC)
    ros_node.create_subscription(
        JointTrajectoryControllerState, ROS_GRIPPER_STATE_TOPIC, self.handle_ros_message, 10)
    ros_node.create_subscription(
        JointTrajectory, ROS_GRIPPER_TRAJECTORY_TOPIC, self.handle_arm_trajectory, 10)

    logging.info("[MQTT] Subscribing to ROS topic %s" %
                 ROS_GRIPPER_STATE_TOPIC)
    logging.info("[MQTT] Subscribing to ROS topic %s" %
                 ROS_GRIPPER_TRAJECTORY_TOPIC)
    self.cur_state = None
    self.desired_position = None
    self.actual_position = None

  def handle_sermas_message(self, msg):
    pass

  def publish(self, state):
    if self.cur_state == state:
      return
    self.cur_state = state
    d = {"state": {"op": ROBOT_STATE_OP_GRIPPER, "state": self.cur_state}}
    logging.info("Robot gripper state: %s" % str(d))
    self.mqtt_client.publish(self.sermas_topic, d)

  def handle_arm_trajectory(self, msg):
    if len(msg.points) == 0:
      return
    pos = msg.points[len(msg.points) - 1].positions
    logging.debug("Requested gripper position: %s" % (str(pos)))
    self.desired_position = pos

  def match_target_position(self, current, target):
    for idx, c in enumerate(current):
      if np.abs(c - target[idx]) > ROBOT_GRIPPER_POSITION_TOLERANCE:
        return False
    return True

  def handle_ros_message(self, msg):
    if self.desired_position is None:
      return
    self.actual_position = msg.actual.positions
    match = self.match_target_position(
        self.desired_position, self.actual_position)
    if match:
      self.publish(ROBOT_STATE_FINISHED)
    else:
      self.publish(ROBOT_STATE_STARTED)
