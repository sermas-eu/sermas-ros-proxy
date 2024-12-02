
import logging
import paho.mqtt.client as mqtt
from paho.mqtt.subscribeoptions import SubscribeOptions
import requests
import time
import threading
import json
import ssl
import os
import sys
from urllib.parse import urlparse
import uuid

TOKEN_PATH = "/api/platform/token/access_token"
REFRESH_PATH = "/api/platform/token/refresh"

class SermasApiClient:
    def __init__(self, toolkit_url, app_id, client_id, client_secret):
        self.toolkit_url = toolkit_url
        self.app_id = app_id
        self.client_id = client_id
        self.client_secret = client_secret
        self.access_token = ""
        self.refresh_token = ""
        self.refresh_interval = 120
        self.subscriptions = []
        while not self.retrieve_token():
            time.sleep(3)

    def get_token(self):
        return self.access_token
    
    def get_subscriptions(self):
        return self.subscriptions
    
    def set_subscriptions(self, subs):
        self.subscriptions = []
        for s in subs:
            # TODO skip the following topics...right?
            if s.find(":agentId") > -1 or s.find(":sessionId") > -1 or s.find(":storageId") > -1:
                continue
            self.subscriptions.append(s.replace(":appId", self.app_id).replace(":clientId", self.client_id))
    
    def retrieve_subscriptions(self):
        subscriptions_path = f"/api/platform/app/{self.app_id}/client/{self.client_id}/topics"
        logging.info("[API] Retrieving subscriptions from %s" % subscriptions_path)
        try:
            response = requests.get(self.toolkit_url + subscriptions_path, headers={ "Authorization": f"Bearer {self.access_token}"})
            response_json = response.json()
            if "statusCode" in response_json and response_json["statusCode"] != 200:
                logging.error("[API] Failed to retrieve subscriptions, error code: %s" % response_json["statusCode"])
                return
            self.set_subscriptions(response_json)
            logging.info("[API] Subscriptions: %s" % self.subscriptions)
        except requests.exceptions.RequestException as e:
            logging.error("[API] Error retrieving subscriptions, url: %s, appId: %s, clientId: %s, error: %s" 
                          % ( self.toolkit_url, self.app_id, self.client_id, str(e)))

    def retrieve_token(self):
        logging.info("[API] Retrieving access token")
        data = {"appId": self.app_id, "clientId": self.client_id,
                "clientSecret": self.client_secret}
        try:
            url = urlparse(self.toolkit_url)
            self.broker_url = url.hostname
            verify_tls = True
            if os.getenv("ENV") == "development" or url.scheme == "http":
                verify_tls = False
            response = requests.post(self.toolkit_url + TOKEN_PATH, json=data, verify=verify_tls)
            response_json = response.json()
            logging.debug("Token response: %s" % str(response_json))
            if "statusCode" in response_json and response_json["statusCode"] != 201:
                logging.error("[API] Failed to retrieve access token, error code: %s" % response_json["statusCode"])
                return False
            self.access_token = response_json["access_token"]
            if "refresh_token" in response_json:
                self.refresh_token = response_json["refresh_token"]
                threading.Timer(self.refresh_interval, self.refresh).start()
            return True
        except requests.exceptions.RequestException as e:
            logging.error("[API] Error retrieving access token, url: %s, appId: %s, clientId: %s, error: %s" 
                          % ( self.toolkit_url, self.app_id, self.client_id, str(e)))
        return False
    
    def refresh(self):
        logging.info("[API] Refreshing token")
        data = {"accessToken": self.access_token, "refreshToken": self.refresh_token }
        try:
            response = requests.post(self.toolkit_url + REFRESH_PATH, json=data)
            response_json = response.json()
            if "statusCode" in response_json and response_json["statusCode"] != 201:
                logging.error("[API] Failed to refresh token, error code: %s" % response_json["statusCode"])
                return
            logging.info("[API] Got refresh token")
            self.access_token = response_json["access_token"]
            self.refresh_token = response_json["refresh_token"]
            threading.Timer(self.refresh_interval, self.refresh).start()
        except requests.exceptions.RequestException as e:
            logging.error("[API] Error refreshing token, url: %s, appId: %s, clientId: %s, error: %s" 
                          % ( self.toolkit_url, self.app_id, self.client_id, str(e)))
            self.retrieve_token()

    def add_to_monitoring(self, data):
        logging.debug("Add to monitoring")
        try:
            requests.post(self.toolkit_url + '/api/platform/monitoring', json=data, headers={'Authorization': f"Bearer {self.access_token}" })
        except requests.exceptions.RequestException as e:
            logging.error("Add to monitoring error: %s" % (str(e)))


class SermasMQTTClient:
    def __init__(self, broker_address, port, app_id, client_id, api_client, callback):
        mqtt_client_id = uuid.uuid4().hex

        self.client = mqtt.Client(client_id=mqtt_client_id,
            transport="websockets", callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
        if os.getenv("ENV") != "development":
            ssl_context = ssl.create_default_context()
            self.client.tls_set_context(ssl_context)
        else:
            logging.info("Env DEVELOPMENT")
            port = 8080

        self.app_id = app_id
        self.client_id = client_id
        self.api_client = api_client
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = callback
        self.topics = []
        self.update_credentials()
        self.connect(broker_address, port)
        self.client.loop_start()

    def connect(self, url, port):
        try:
            logging.info(f"[MQTT] Connecting to broker {url}:{port}")
            self.client.connect(url, port, keepalive=10)
        except Exception as e:
            logging.error("[MQTT] Failed to connect to broker, error: %s" % e)
            sys.exit(-1)

    def update_credentials(self):
        self.client.username_pw_set(username=self.api_client.get_token(), password="sermas")

    def get_client(self):
        return self.client
    
    def set_topics(self, topics):
        self.topics = topics
        self.subscribe()
    
    def subscribe(self):
        options = SubscribeOptions(qos=1, noLocal=True)
        for s in self.topics:
            topic = f"app/{self.app_id}/{s}"
            logging.info("[MQTT] Subscribing to SERMAS topic %s" % topic)
            self.client.subscribe(f"app/{self.app_id}/{s}", options=options)

    def unsubscribe(self):
        for s in self.topics:
            topic = f"app/{self.app_id}/{s}"
            logging.info("[MQTT] Unsubscribing to SERMAS topic %s" % topic)
            self.client.unsubscribe(f"app/{self.app_id}/{s}")
     
    def on_connect(self, client, userdata, flags, rc):
        if rc==0:
            # client.connected_flag=True #set flag
            logging.info("[MQTT] Connected to SERMAS toolkit")
            self.subscribe()
        else:
            logging.info("[MQTT] Bad connection to SERMAS toolkit: " + str(rc))
            self.update_credentials()

    def on_disconnect(self, client, userdata, rc):
        logging.error("[MQTT] Disconnected from SERMAS toolkit: " + str(rc))
        if rc == 4:
            logging.error("Reason: bad username or password")
        elif rc == 5:
            logging.error("Reason: not authorized")
        self.update_credentials()
        self.unsubscribe()

    def publish(self, topic, message):
        message["appId"] = self.app_id
        message["clientId"] = self.client_id
        sermas_topic = f"app/{self.app_id}/{topic}"
        try:
            self.client.publish(sermas_topic, json.dumps(message))
            logging.debug("[MQTT] published %s to %s" % (
                json.dumps(message), sermas_topic))
        except Exception as e:
           logging.error("[MQTT] Failed to publish, error: %s" % e)
