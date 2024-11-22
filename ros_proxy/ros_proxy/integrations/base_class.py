from abc import ABC, abstractmethod
import logging
class IntegrationBaseClass(ABC):
  def __init__(self, ros_node, sermas_topic=""):
    self.ros_node = ros_node
    self.mqtt_client = ros_node.mqtt_client
    self.api_client = ros_node.api_client
    self.sermas_topic = sermas_topic

  def handle_mqtt_message(self, client, userdata, msg):
    topic = f"app/{self.ros_node.app_id}/{self.sermas_topic}".replace("+","")
    if msg.topic[:len(topic)] == topic:
      self.handle_sermas_message(msg)

  @abstractmethod
  def handle_sermas_message(self, msg):
    raise NotImplementedError()

