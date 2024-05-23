from enum import Enum
import logging
import json
import os
from abc import ABC, abstractmethod
import time
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from users_landmarks_msgs.msg import MultipleUsersLandmarks
from mutual_gaze_detector_msgs.msg import MutualGazeOutput


INTENTION_PROBABILITY_THRESHOLD = int(os.environ.get(
    "INTENTION_PROBABILITY_THRESHOLD", 0.6))
INTENTION_DISTANCE_THRESHOLD = int(os.environ.get(
    "INTENTION_DISTANCE_THRESHOLD", 1.5))
MIN_SENDING_INTERVAL_SEC = int(os.environ.get("MIN_SENDING_INTERVAL_SEC", 0.5))

ROS_USERS_LANDMARKS_TOPIC = os.environ.get(
    "ROS_USERS_LANDMARKS_TOPIC", "/face_landmarks_node/users_landmarks")
ROS_MUTUAL_GAZE_TOPIC = os.environ.get(
    "ROS_MUTUAL_GAZE_TOPIC", "/mutual_gaze_output")
ROS_USER_POSITION_TOPIC = os.environ.get(
    "ROS_USER_POSITION_TOPIC", "/user/position")
ROS_ODOMETRY_TOPIC = os.environ.get("ROS_ODOMETRY_TOPIC", "/odom")
ROS_GOALPOSE_TOPIC = os.environ.get(
    "ROS_GOALPOSE_TOPIC", "/move_base_simple/goal")
ROS_INITIALPOSE_TOPIC = os.environ.get("ROS_INITIALPOSE_TOPIC", "/initialpose")
ROS_VELOCITY_TOPIC = os.environ.get("ROS_VELOCITY_TOPIC", "/cmd_vel")
ROS_VIDEO_TOPIC = os.environ.get(
    "ROS_VIDEO_TOPIC", "/v4l/camera/image_raw/compressed")

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

class TopicDirection(Enum):
  ROS_TO_PLATFORM = 0
  PLAFTORM_TO_ROS = 1

class BaseTopic(ABC):
  def __init__(self, ros_node, mqtt_client, direction: TopicDirection, ros_topic = "", sermas_topic = ""):
    self.ros_node = ros_node
    self.direction = direction
    self.sermas_topic = sermas_topic
    self.mqtt_client = mqtt_client
  
  def handle_mqtt_message(self, client, userdata, msg):
    if msg.topic == f"app/{self.ros_node.app_id}/{self.sermas_topic}":
      self.handle_sermas_message(msg)
  
  @abstractmethod
  def handle_ros_message(self, msg):
    raise NotImplementedError()
  
  @abstractmethod
  def handle_sermas_message(self, msg):
    raise NotImplementedError()


"""
Forward user and intent detection to SERMAS toolkit
"""
class BodyTracking(BaseTopic):
  def __init__(self, ros_node, mqtt_client):
    super().__init__(ros_node, mqtt_client, TopicDirection.ROS_TO_PLATFORM, ROS_USERS_LANDMARKS_TOPIC, SERMAS_USER_DETECTION_TOPIC)
    self.mutual_haze = {}
    self.last_ts = 0
    ros_node.create_subscription(MultipleUsersLandmarks, ROS_USERS_LANDMARKS_TOPIC, self.handle_ros_message, 10)
    ros_node.create_subscription(
        MutualGazeOutput, ROS_MUTUAL_GAZE_TOPIC, self.handle_ros_mutual_haze_message, 10)
    logging.info("[MQTT] Subscribing to ROS topic %s" % ROS_USERS_LANDMARKS_TOPIC)

  def handle_sermas_message(self, msg):
    pass

  def handle_ros_mutual_haze_message(self, msg):
    for index, id in enumerate(msg.body_ids):
      self.mutual_haze[id] = msg.output[index]

  def get_prob(self, body_id):
    if body_id in self.mutual_haze:
      return self.mutual_haze[body_id]
    return 0

  def handle_ros_message(self, msg):
    detections = []
    for u in msg.users:
      if len(u.body_landmarks) < 4:
        continue
      l = u.body_landmarks[3]
      logging.debug("Found NEK marker, distance: %.2f meters" % l.position.z)
      detections.append({"user": {"value": u.body_id, "probability": self.get_prob(
          u.body_id)}, "position": {"x": l.position.x, "y": l.position.y, "z": l.position.z}})
    if len(detections) > 0:
      '''
      Publish user detection list
      '''
      logging.debug("Detected people: %d" % len(detections))
      d = { "cameraId": "kinect", "source": "kinect", "detections": detections }
      self.mqtt_client.publish(self.sermas_topic, d)
      '''
      Publish intent detection list only if user is close enough
      '''
      for d in detections:
        prob = d["user"]["probability"]
        dist = d["position"]["z"]
        if prob > INTENTION_PROBABILITY_THRESHOLD and dist < INTENTION_DISTANCE_THRESHOLD and self.last_ts < (time.time() - MIN_SENDING_INTERVAL_SEC):
          self.last_ts = time.time()
          d = {"moduleId": "detection", "source": "camera",
               "probability": d["user"]["probability"], "interactionType": "start", "sessionId": ""}
          self.mqtt_client.publish(SERMAS_INTENT_DETECTION_TOPIC, d)
          logging.info("Intent detection: %s" % str(d))


"""
Forward user detection data from SERMAS toolkit to ROS
"""
class UserPosition(BaseTopic):
  def __init__(self, ros_node, mqtt_client):
    super().__init__(ros_node, mqtt_client, TopicDirection.PLAFTORM_TO_ROS, ROS_USER_POSITION_TOPIC, SERMAS_USER_DETECTION_TOPIC)
    self.publisher = self.ros_node.create_publisher(Pose, ROS_USER_POSITION_TOPIC, 10)

  def handle_ros_message(self, msg):
    pass
  
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


"""
Forward Robot position and velocity to SERMAS toolkit
"""
class RobotStatus(BaseTopic):
  def __init__(self, ros_node, mqtt_client):
    super().__init__(ros_node, mqtt_client, TopicDirection.ROS_TO_PLATFORM, ROS_ODOMETRY_TOPIC, SERMAS_ROBOT_STATUS_TOPIC)
    ros_node.create_subscription(Odometry, ROS_ODOMETRY_TOPIC, self.handle_ros_message, 10)
    logging.info("[MQTT] Subscribing to ROS topic %s" % ROS_ODOMETRY_TOPIC)

  def handle_sermas_message(self, msg):
    pass

  def handle_ros_message(self, msg):
    d = { "status":
          { "actualPosition": { "position":{ "x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y, "z": msg.pose.pose.position.z},
                               "orientation": { "x": msg.pose.pose.orientation.x, "y": msg.pose.pose.orientation.y, "z": msg.pose.pose.orientation.z, "w": msg.pose.pose.orientation.w}},
          "velocity": { 
             "linear": { "x": msg.twist.twist.linear.x, "y": msg.twist.twist.linear.y, "z": msg.twist.twist.linear.z },
             "angular": { "x": msg.twist.twist.angular.x, "y": msg.twist.twist.angular.y, "z": msg.twist.twist.angular.z } 
             }
          }
        }
    logging.debug("Robot status: %s" % str(d))
    self.mqtt_client.publish(self.sermas_topic, d)


"""
Forward Robot commands from SERMAS toolkit to ROS
"""
class RobotCmd(BaseTopic):
  def __init__(self, ros_node, mqtt_client):
    super().__init__(ros_node, mqtt_client, TopicDirection.PLAFTORM_TO_ROS, ROS_GOALPOSE_TOPIC, SERMAS_ROBOTCMD_TOPIC)
    self.publisher = ros_node.create_publisher(PoseStamped, ROS_GOALPOSE_TOPIC, 10)

  def handle_ros_message(self, msg):
    pass

  def handle_sermas_message(self, msg):
    logging.info("Send robot cmd (topic: %s): %s" % (ROS_GOALPOSE_TOPIC ,str(msg.payload.decode())))
    d = json.loads(str(msg.payload.decode()))
    ros_msg = PoseStamped()
    ros_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
    ros_msg.header.frame_id = 'map'
    ros_msg.pose.position.x = float(d["movement"]["targetPosition"]["position"]["x"])
    ros_msg.pose.position.y = float(d["movement"]["targetPosition"]["position"]["y"])
    ros_msg.pose.position.z = float(d["movement"]["targetPosition"]["position"]["z"])
    ros_msg.pose.orientation.x = float(d["movement"]["targetPosition"]["orientation"]["x"])
    ros_msg.pose.orientation.y = float(d["movement"]["targetPosition"]["orientation"]["y"])
    ros_msg.pose.orientation.z = float(d["movement"]["targetPosition"]["orientation"]["z"])
    ros_msg.pose.orientation.w = float(d["movement"]["targetPosition"]["orientation"]["w"])
    self.publisher.publish(ros_msg)


"""
Forward Robot initial position from SERMAS toolkit to ROS
"""
class RobotInitialPose(BaseTopic):
  def __init__(self, ros_node, mqtt_client):
    super().__init__(ros_node, mqtt_client, TopicDirection.PLAFTORM_TO_ROS, ROS_INITIALPOSE_TOPIC, SERMAS_ROBOTINITIALPOSE_TOPIC)
    self.publisher = ros_node.create_publisher(PoseWithCovarianceStamped, ROS_INITIALPOSE_TOPIC, 10)

  def handle_ros_message(self, msg):
    pass

  def handle_sermas_message(self, msg):
    logging.info("Send robot initial pose (topic: %s): %s" % (ROS_INITIALPOSE_TOPIC ,str(msg.payload.decode())))
    d = json.loads(str(msg.payload.decode()))
    ros_msg = PoseWithCovarianceStamped()
    ros_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
    ros_msg.header.frame_id = 'map'
    ros_msg.pose.pose.position.x = float(d["initialPose"]["pose"]["position"]["x"])
    ros_msg.pose.pose.position.y = float(d["initialPose"]["pose"]["position"]["y"])
    ros_msg.pose.pose.position.z = float(d["initialPose"]["pose"]["position"]["z"])
    ros_msg.pose.pose.orientation.x = float(d["initialPose"]["pose"]["orientation"]["x"])
    ros_msg.pose.pose.orientation.y = float(d["initialPose"]["pose"]["orientation"]["y"])
    ros_msg.pose.pose.orientation.z = float(d["initialPose"]["pose"]["orientation"]["z"])
    ros_msg.pose.pose.orientation.w = float(d["initialPose"]["pose"]["orientation"]["w"])
    ros_msg.pose.covariance = [float(v) for v in d["initialPose"]["covariance"]]
    self.publisher.publish(ros_msg)


"""
Forward Robot velocity to SERMAS toolkit
"""
class RobotVelocity(BaseTopic):
  def __init__(self, ros_node, mqtt_client):
    super().__init__(ros_node, mqtt_client, TopicDirection.ROS_TO_PLATFORM, ROS_VELOCITY_TOPIC, SERMAS_ROBOT_STATUS_TOPIC)
    ros_node.create_subscription(Twist, ROS_VELOCITY_TOPIC, self.handle_ros_message, 10)
    logging.info("[MQTT] Subscribing to ROS topic %s" % ROS_VELOCITY_TOPIC)

  def handle_sermas_message(self, msg):
    pass

  def handle_ros_message(self, msg):
    d = { "status":
          {
              # { "actualPosition": { "position":{ "x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y, "z": msg.pose.pose.position.z},
              #                    "orientation": { "x": msg.pose.pose.orientation.x, "y": msg.pose.pose.orientation.y, "z": msg.pose.pose.orientation.z, "w": msg.pose.pose.orientation.w}
              #                     },
              "velocity": {
            "linear": { "x": msg.linear.x, "y": msg.linear.y, "z": msg.linear.z },
            "angular": { "x": msg.angular.x, "y": msg.angular.y, "z": msg.angular.z }
            }
          }
        }
    logging.debug("Robot status: %s" % str(d))
    self.mqtt_client.publish(self.sermas_topic, d)


"""
Forward video feed to SERMAS toolkit
"""


class VideoFeed(BaseTopic):
  def __init__(self, ros_node, mqtt_client):
    super().__init__(ros_node, mqtt_client, TopicDirection.ROS_TO_PLATFORM,
                     ROS_VIDEO_TOPIC, SERMAS_ROBOT_VIDEO_FEED_TOPIC)
    ros_node.create_subscription(
        Image, ROS_VIDEO_TOPIC, self.handle_ros_message, 10)
    logging.info("[MQTT] Subscribing to ROS topic %s" % ROS_VIDEO_TOPIC)
    self.bridge = CvBridge()

  def handle_sermas_message(self, msg):
    pass

  def handle_ros_message(self, msg):
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    _, img_encoded = cv2.imencode('.jpg', cv_image)
    d = {"data": img_encoded.tostring()}
    # logging.debug("Robot status: %s" % str(d))
    self.mqtt_client.publish(self.sermas_topic, d)
