import logging
import json
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from ros_proxy.ros_proxy.config.topics import ROS_GOALPOSE_TOPIC, ROS_INITIALPOSE_TOPIC, ROS_ODOMETRY_TOPIC, ROS_VELOCITY_TOPIC, SERMAS_ROBOT_STATUS_TOPIC, SERMAS_ROBOTCMD_TOPIC, SERMAS_ROBOTINITIALPOSE_TOPIC
from ros_proxy.ros_proxy.sermas_topics import IntegrationBaseClass


"""
Forward Robot position and velocity to SERMAS toolkit
"""


class MyAgvRobotStatus(IntegrationBaseClass):
  def __init__(self, ros_node):
    super().__init__(ros_node, SERMAS_ROBOT_STATUS_TOPIC)
    ros_node.create_subscription(
        Odometry, ROS_ODOMETRY_TOPIC, self.handle_ros_message, 10)
    logging.info("[MQTT] Subscribing to ROS topic %s" % ROS_ODOMETRY_TOPIC)

  def handle_sermas_message(self, msg):
    pass

  def handle_ros_message(self, msg):
    d = {"status":
         {"actualPosition": {"position": {"x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y, "z": msg.pose.pose.position.z},
                             "orientation": {"x": msg.pose.pose.orientation.x, "y": msg.pose.pose.orientation.y, "z": msg.pose.pose.orientation.z, "w": msg.pose.pose.orientation.w}},
          "velocity": {
             "linear": {"x": msg.twist.twist.linear.x, "y": msg.twist.twist.linear.y, "z": msg.twist.twist.linear.z},
              "angular": {"x": msg.twist.twist.angular.x, "y": msg.twist.twist.angular.y, "z": msg.twist.twist.angular.z}
         }
         }
         }
    logging.debug("Robot status: %s" % str(d))
    self.mqtt_client.publish(self.sermas_topic, d)


"""
Forward Robot commands from SERMAS toolkit to ROS
"""


class MyAgvRobotCmd(IntegrationBaseClass):
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


"""
Forward Robot initial position from SERMAS toolkit to ROS
"""


class MyAgvRobotInitialPose(IntegrationBaseClass):
  def __init__(self, ros_node):
    super().__init__(ros_node, SERMAS_ROBOTINITIALPOSE_TOPIC)
    self.publisher = ros_node.create_publisher(
        PoseWithCovarianceStamped, ROS_INITIALPOSE_TOPIC, 10)

  def handle_ros_message(self, msg):
    pass

  def handle_sermas_message(self, msg):
    logging.info("Send robot initial pose (topic: %s): %s" %
                 (ROS_INITIALPOSE_TOPIC, str(msg.payload.decode())))
    d = json.loads(str(msg.payload.decode()))
    ros_msg = PoseWithCovarianceStamped()
    ros_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
    ros_msg.header.frame_id = 'map'
    ros_msg.pose.pose.position.x = float(
        d["initialPose"]["pose"]["position"]["x"])
    ros_msg.pose.pose.position.y = float(
        d["initialPose"]["pose"]["position"]["y"])
    ros_msg.pose.pose.position.z = float(
        d["initialPose"]["pose"]["position"]["z"])
    ros_msg.pose.pose.orientation.x = float(
        d["initialPose"]["pose"]["orientation"]["x"])
    ros_msg.pose.pose.orientation.y = float(
        d["initialPose"]["pose"]["orientation"]["y"])
    ros_msg.pose.pose.orientation.z = float(
        d["initialPose"]["pose"]["orientation"]["z"])
    ros_msg.pose.pose.orientation.w = float(
        d["initialPose"]["pose"]["orientation"]["w"])
    ros_msg.pose.covariance = [float(v)
                               for v in d["initialPose"]["covariance"]]
    self.publisher.publish(ros_msg)


"""
Forward Robot velocity to SERMAS toolkit
"""


class MyAgvRobotVelocity(IntegrationBaseClass):
  def __init__(self, ros_node):
    super().__init__(ros_node, SERMAS_ROBOT_STATUS_TOPIC)
    ros_node.create_subscription(
        Twist, ROS_VELOCITY_TOPIC, self.handle_ros_message, 10)
    logging.info("[MQTT] Subscribing to ROS topic %s" % ROS_VELOCITY_TOPIC)

  def handle_sermas_message(self, msg):
    pass

  def handle_ros_message(self, msg):
    d = {"status":
         {
             # { "actualPosition": { "position":{ "x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y, "z": msg.pose.pose.position.z},
             #                    "orientation": { "x": msg.pose.pose.orientation.x, "y": msg.pose.pose.orientation.y, "z": msg.pose.pose.orientation.z, "w": msg.pose.pose.orientation.w}
             #                     },
             "velocity": {
                 "linear": {"x": msg.linear.x, "y": msg.linear.y, "z": msg.linear.z},
                 "angular": {"x": msg.angular.x, "y": msg.angular.y, "z": msg.angular.z}
             }
         }
         }
    logging.debug("Robot status: %s" % str(d))
    self.mqtt_client.publish(self.sermas_topic, d)
