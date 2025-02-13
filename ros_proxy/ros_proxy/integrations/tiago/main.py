import logging
import json
from geometry_msgs.msg import PoseStamped
from ros_proxy.config.topics import ROS_GOALPOSE_TOPIC, ROS_USER_POSITION_TOPIC, SERMAS_ROBOTCMD_TOPIC, SERMAS_USER_DETECTION_TOPIC
from ros_proxy.integrations.base_class import IntegrationBaseClass


"""
Forward move command data from SERMAS toolkit to ROS
"""

class TiagoRobotCmd(IntegrationBaseClass):
  def __init__(self, ros_node):
    super().__init__(ros_node, SERMAS_ROBOTCMD_TOPIC)
    self.publisher = ros_node.create_publisher(
        PoseStamped, ROS_GOALPOSE_TOPIC, 10)

  def handle_ros_message(self, msg):
    pass

  def handle_sermas_message(self, msg):
    logging.info("Send robot cmd (topic: %s): %s" %
                 (ROS_GOALPOSE_TOPIC, str(msg.payload.decode())))
    d = json.loads(str(msg.payload.decode()))
    ros_msg = PoseStamped()
    ros_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
    ros_msg.header.frame_id = 'map'
    ros_msg.pose.position.x = float(
        d["movement"]["targetPosition"]["position"]["x"])
    ros_msg.pose.position.y = float(
        d["movement"]["targetPosition"]["position"]["y"])
    ros_msg.pose.position.z = float(
        d["movement"]["targetPosition"]["position"]["z"])
    ros_msg.pose.orientation.x = float(
        d["movement"]["targetPosition"]["orientation"]["x"])
    ros_msg.pose.orientation.y = float(
        d["movement"]["targetPosition"]["orientation"]["y"])
    ros_msg.pose.orientation.z = float(
        d["movement"]["targetPosition"]["orientation"]["z"])
    ros_msg.pose.orientation.w = float(
        d["movement"]["targetPosition"]["orientation"]["w"])
    self.publisher.publish(ros_msg)