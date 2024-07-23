import logging
import os
import rclpy

from ros_proxy.config.topics import ROS_SESSION_TOPIC, ROS_MUTUAL_GAZE_TOPIC, ROS_USERS_LANDMARKS_TOPIC, SERMAS_INTENT_DETECTION_TOPIC, SERMAS_USER_DETECTION_TOPIC
from ros_proxy.integrations.base_class import IntegrationBaseClass
# from users_landmarks_msgs.msg import MultipleUsersLandmarks
# from mutual_gaze_detector_msgs.msg import MutualGazeOutput

from session_event_msgs.msg import SessionEvent


INTENTION_PROBABILITY_THRESHOLD = float(os.environ.get(
    "INTENTION_PROBABILITY_THRESHOLD", 0.5))
INTENTION_DISTANCE_THRESHOLD = float(os.environ.get(
    "INTENTION_DISTANCE_THRESHOLD", 2.0))
MIN_SENDING_INTERVAL_SEC = float(
    os.environ.get("MIN_SENDING_INTERVAL_SEC", 0.5))

"""
Forward user and intent detection to SERMAS toolkit
"""


class IntentDetection(IntegrationBaseClass):
  def __init__(self, ros_node):
    super().__init__(ros_node, SERMAS_INTENT_DETECTION_TOPIC)
    self.qos_pub = rclpy.qos.QoSProfile(
        depth=1,
    )
    self.ros_node.create_subscription(
        SessionEvent, ROS_SESSION_TOPIC, self.handle_ros_session_message, self.qos_pub)
    logging.info("[MQTT] Subscribing to ROS topic %s" %
                 ROS_SESSION_TOPIC)

  def handle_sermas_message(self, msg):
    pass

  def handle_ros_session_message(self, msg):
    logging.info("Session event %s for userId %d" %
                 (msg.event_type, msg.user_id))
    d = {"moduleId": "detection", "source": "camera", "userId": str(msg.user_id),
         "probability": 1, "interactionType": msg.event_type, "sessionId": ""}
    self.ros_node.mqtt_client.publish(self.sermas_topic, d)


# class Kinect(IntegrationBaseClass):
#   def __init__(self, ros_node):
#     super().__init__(ros_node, SERMAS_USER_DETECTION_TOPIC)
#     self.mutual_haze = {}
#     self.last_ts = 0
#     self.ros_node.create_subscription(
#         MultipleUsersLandmarks, ROS_USERS_LANDMARKS_TOPIC, self.handle_ros_landmark_message, 10)
#     self.ros_node.create_subscription(
#         MutualGazeOutput, ROS_MUTUAL_GAZE_TOPIC, self.handle_ros_mutual_haze_message, 10)
#     logging.info("[MQTT] Subscribing to ROS topic %s" %
#                  ROS_USERS_LANDMARKS_TOPIC)

#   def handle_sermas_message(self, msg):
#     pass

#   def handle_ros_mutual_haze_message(self, msg):
#     if len(msg.body_ids) != len(msg.output):
#       logging.error('Mutual gaze output and body_ids len mismatch [%d != %d]' % (
#           len(msg.body_ids), len(msg.output)))
#       return
#     for index, id in enumerate(msg.body_ids):
#       self.mutual_haze[id] = msg.output[index]

#   def get_prob(self, body_id):
#     if body_id in self.mutual_haze:
#       return self.mutual_haze[body_id]
#     return 0

#   def handle_ros_landmark_message(self, msg):
#     detections = []
#     for u in msg.users:
#       if len(u.body_landmarks) < 32:
#         logging.warn("Less than 32 (%d) detection landmarks" %
#                      len(u.body_landmarks))
#         continue
#       l = u.body_landmarks[3]
#       logging.debug("Found NEK marker, distance: %.2f meters" % l.position.z)
#       detections.append({"user": {"value": u.body_id, "probability": self.get_prob(
#           u.body_id)}, "position": {"x": l.position.x, "y": l.position.y, "z": l.position.z}})
#     if len(detections) > 0:
#       '''
#       Publish user detection list
#       '''
#       logging.debug("Detected people: %d" % len(detections))
#       d = {"cameraId": "kinect", "source": "kinect", "detections": detections}
#       self.ros_node.mqtt_client.publish(self.sermas_topic, d)
#       '''
#       Publish intent detection list only if user is close enough
#       '''
#       for d in detections:
#         prob = d["user"]["probability"]
#         dist = d["position"]["z"]
#         if prob > INTENTION_PROBABILITY_THRESHOLD and dist < INTENTION_DISTANCE_THRESHOLD and self.last_ts < (time.time() - MIN_SENDING_INTERVAL_SEC):
#           self.last_ts = time.time()
#           d = {"moduleId": "detection", "source": "camera",
#                "probability": d["user"]["probability"], "interactionType": "start", "sessionId": ""}
#           self.ros_node.mqtt_client.publish(SERMAS_INTENT_DETECTION_TOPIC, d)
#           logging.info("Intent detection: %s" % str(d))
