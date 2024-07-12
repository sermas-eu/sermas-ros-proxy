import logging
import json
from geometry_msgs.msg import Pose
from ros_proxy.config.topics import ROS_USER_POSITION_TOPIC, SERMAS_USER_DETECTION_TOPIC
from ros_proxy.integrations.base_class import IntegrationBaseClass


"""
Forward user detection data from SERMAS toolkit to ROS
"""


class Tiago(IntegrationBaseClass):
  def __init__(self, ros_node):
    super().__init__(ros_node, SERMAS_USER_DETECTION_TOPIC)
    self.publisher = self.ros_node.create_publisher(
        Pose, ROS_USER_POSITION_TOPIC, 10)

  def handle_mqtt_message(self, client, userdata, msg):
    if msg.topic == f"app/{self.ros_node.app_id}/{self.sermas_topic}":
      self.handle_sermas_message(msg)

  def handle_sermas_message(self, msg):
    logging.debug("Received user detection data %s" % str(msg.payload))
    d = json.loads(str(msg.payload.decode()))
    if len(d["detections"]) == 0:
      return
    ros_msg = Pose()
    ros_msg.position.x = d["detections"][0]["position"]["x"]
    ros_msg.position.y = d["detections"][0]["position"]["y"]
    ros_msg.position.z = d["detections"][0]["position"]["z"]
    self.publisher.publish(ros_msg)
