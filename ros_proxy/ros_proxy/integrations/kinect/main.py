import logging
import os
import rclpy
import time
import datetime

from ros_proxy.config.topics import ROS_SESSION_TOPIC, ROS_USERS_LANDMARKS_TOPIC, SERMAS_INTENT_DETECTION_TOPIC, SERMAS_ROBOT_STATUS_TOPIC, SERMAS_USER_DETECTION_TOPIC
from ros_proxy.integrations.base_class import IntegrationBaseClass
# from users_landmarks_msgs.msg import MultipleUsersLandmarks
# from mutual_gaze_detector_msgs.msg import MutualGazeOutput

from session_event_msgs.msg import SessionEvent
from users_landmarks_msgs.msg import MultipleUsersLandmarks

INTENTION_PROBABILITY_THRESHOLD = float(os.environ.get(
    "INTENTION_PROBABILITY_THRESHOLD", 0.5))
INTENTION_DISTANCE_THRESHOLD = float(os.environ.get(
    "INTENTION_DISTANCE_THRESHOLD", 2.0))
MIN_SENDING_INTERVAL_SEC = float(
    os.environ.get("MIN_SENDING_INTERVAL_SEC", 0.5))

MIN_VALIDITY_INTERVAL_SEC = float(os.environ.get("MIN_VALIDITY_INTERVAL_SEC", 5.0))

"""
Forward user and intent detection to SERMAS toolkit
"""


class IntentDetection(IntegrationBaseClass):
  def __init__(self, ros_node):
    super().__init__(ros_node, SERMAS_INTENT_DETECTION_TOPIC)
    self.last_landmark_ts = 0
    self.data_available = False
    self.sermas_status_topic = SERMAS_ROBOT_STATUS_TOPIC
    self.qos_pub = rclpy.qos.QoSProfile(
        depth=1,
    )
    self.ros_node.create_subscription(
        SessionEvent, ROS_SESSION_TOPIC, self.handle_ros_session_message, self.qos_pub)
    self.ros_node.create_subscription(
        MultipleUsersLandmarks, ROS_USERS_LANDMARKS_TOPIC, self.handle_landmarks, self.qos_pub)
    logging.info("[MQTT] Subscribing to ROS topics [%s,%s]" %
                 (ROS_SESSION_TOPIC,ROS_USERS_LANDMARKS_TOPIC))
    self.ros_node.create_timer(5.0, self.check_status)

  def add_to_monitoring(self, event_type: str, data: str):
    now = datetime.datetime.now()
    nowStr = now.isoformat()
    d = {"appId": self.ros_node.app_id, "sessionId": "", "type": event_type, 
         "label": data, "ts": nowStr}
    self.ros_node.mqtt_client.publish('platform/monitoring', d)

  def handle_sermas_message(self, msg):
    pass

  def handle_landmarks(self, msg):
    self.last_landmark_ts = time.time()

  def mapInteractionType(self, event_type):
    if event_type == 'start':
      return 'start'
    return 'stop'

  def handle_ros_session_message(self, msg):
    logging.info("Session event %s for userId %d" %
                 (msg.event_type, msg.user_id))
    d = {"moduleId": "detection", "source": "camera", "userId": str(msg.user_id),
         "probability": 1, "interactionType": self.mapInteractionType(msg.event_type), "sessionId": ""}
    self.ros_node.mqtt_client.publish(self.sermas_topic, d)
    self.add_to_monitoring("interaction", f"Intent detection event type: {msg.event_type}, userId: {msg.user_id}")

  def check_status(self):
    if self.last_landmark_ts == 0:
      # just started
      return
    if (time.time() - self.last_landmark_ts) > MIN_VALIDITY_INTERVAL_SEC:
      if self.data_available:
            # if changing to false send an error
            self.add_to_monitoring("error", f"Intent detection not available")
      self.data_available = False
    else:
      self.data_available = True
    logging.info("Device status: %s" % ("available" if self.data_available else "unavailable"))
    d = { "status": 
          {
            "available": "true" if self.data_available else "false"
          }
        }
    self.mqtt_client.publish(self.sermas_status_topic, d)

