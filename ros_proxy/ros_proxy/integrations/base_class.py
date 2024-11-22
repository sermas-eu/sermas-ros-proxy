from abc import ABC, abstractmethod
import datetime

class IntegrationBaseClass(ABC):
  def __init__(self, ros_node, sermas_topic=""):
    self.ros_node = ros_node
    self.mqtt_client = ros_node.mqtt_client
    self.sermas_topic = sermas_topic

  def handle_mqtt_message(self, client, userdata, msg):
    if msg.topic == f"app/{self.ros_node.app_id}/{self.sermas_topic}":
      self.handle_sermas_message(msg)

  @abstractmethod
  def handle_sermas_message(self, msg):
    raise NotImplementedError()
  
  def add_to_monitoring(self, type: str, data: str):
    d = {"appId": self.ros_node.app_id, "sessionId": "", "type": type, "label": data, "ts": datetime.datetime.now()}
    self.ros_node.mqtt_client.publish('platform/monitoring', d)
