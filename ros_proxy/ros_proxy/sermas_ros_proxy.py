from ros_proxy.ros_proxy.integrations.kinect.main import Kinect
from ros_proxy.ros_proxy.integrations.tiago.main import Tiago
from sermas_clients import SermasApiClient,SermasMQTTClient
import rclpy
from rclpy.node import Node
import logging
import os
from urllib.parse import urlparse

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()

logging.basicConfig(format='%(asctime)s %(levelname)-8s %(message)s',
                    level=LOGLEVEL, datefmt='%Y-%m-%d %H:%M:%S')

class SermasRosProxy(Node):
    def __init__(self):
        super().__init__('sermas_ros_proxy')
        self.load_config()
        api_client = SermasApiClient(self.api_url, self.app_id, self.client_id, self.client_secret)
        self.mqtt_client = SermasMQTTClient(self.broker_url, self.broker_port, self.app_id, 
                                            self.client_id, api_client, self.handle_sermas_message)
        self.integrations = []
        # self.topics = [BodyTracking(self, self.mqtt_client),
        #                UserPosition(self, self.mqtt_client),
        #                RobotStatus(self, self.mqtt_client),
        #                RobotCmd(self, self.mqtt_client),
        #                RobotInitialPose(self, self.mqtt_client),
        #                RobotActuate(self, self.mqtt_client),
        #                RobotArmState(self, self.mqtt_client),
        #                RobotGripperState(self, self.mqtt_client)]
        #    RobotVelocity(self, self.mqtt_client)]
        #    VideoFeed(self, self.mqtt_client)]
        subs = [t.sermas_topic for t in self.integrations if t.sermas_topic != ""]
        self.mqtt_client.set_topics(subs)
    #     self.test_kinect()

    # def test_kinect(self):
    #     d = {"moduleId": "detection", "source": "camera",
    #          "probability": 0.9, "interactionType": "start", "sessionId": ""}
    #     self.mqtt_client.publish("detection/interaction", d)

    def load_integrations(self):
        self.integrations.append(Kinect(self, self.mqtt_client))
        self.integrations.append(Tiago(self, self.mqtt_client))

    def ensure_env(self, env):
        if not env in os.environ:
            logging.error(f"Missing env '{env}' !!!")
            exit(1)
        logging.info(f"Env {env}: {os.getenv(env)}")
        return os.getenv(env)

    def load_config(self):
        self.api_url = self.ensure_env("SERMAS_TOOLKIT_URL")
        url = urlparse(self.api_url)
        self.broker_url = url.hostname
        if url.scheme == "https":
            self.broker_port = 443
        else:
            self.broker_port = 1884
        self.client_id = self.ensure_env("CLIENT_ID") 
        self.client_secret = self.ensure_env("CLIENT_SECRET") 
        self.app_id = self.ensure_env("APP_ID")
        if os.getenv("ENV") == "development":
            logging.info("Env DEVELOPMENT")
            self.broker_url = 'localhost'
            self.broker_port = 1883

    def handle_sermas_message(self, client, userdata, msg):
        for t in self.integrations:
            t.handle_mqtt_message(client, userdata, msg)


def main(args=None):
    rclpy.init(args=args)
    proxy = SermasRosProxy()
    rclpy.spin(proxy)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    proxy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
