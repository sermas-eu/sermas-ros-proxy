import rclpy
from rclpy.node import Node
import logging
import os
from urllib.parse import urlparse

from ros_proxy.integrations.kinect.main import IntentDetection
from ros_proxy.integrations.myagv.main import MyAgvRobotCmd, MyAgvRobotStatus
from ros_proxy.integrations.mycobot.main import MyCobotRobotActuate, MyCobotRobotArmState, MyCobotRobotGripperState
from ros_proxy.integrations.tiago.main import TiagoRobotCmd
from sermas_clients import SermasApiClient, SermasMQTTClient

LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()
logging.basicConfig(format='%(asctime)s %(levelname)-8s %(message)s',
                    level=LOGLEVEL, datefmt='%Y-%m-%d %H:%M:%S')

class SermasRosProxy(Node):
    def __init__(self):
        super().__init__('sermas_ros_proxy')
        self.load_config()
        self.api_client = SermasApiClient(self.api_url, self.app_id, self.client_id, self.client_secret)
        self.mqtt_client = SermasMQTTClient(self.broker_url, self.broker_port, self.app_id, 
                                            self.client_id, self.api_client, self.handle_sermas_message)
        self.load_integrations()
        subs = [t.sermas_topic for t in self.integrations if t.sermas_topic != ""]
        self.mqtt_client.set_topics(subs)

    def load_integrations(self):
        self.integrations = []
        self.integrations.append(IntentDetection(self))
        self.integrations.append(TiagoRobotCmd(self))
        # self.integrations.append(MyAgvRobotStatus(self))
        # self.integrations.append(MyAgvRobotCmd(self))
        # self.integrations.append(MyCobotRobotActuate(self))
        # self.integrations.append(MyCobotRobotArmState(self))
        # self.integrations.append(MyCobotRobotGripperState(self))

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
