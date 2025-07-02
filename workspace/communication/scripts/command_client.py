#usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import string
from rclpy import int
from base_socket import ConnectionBase


class Publisher(Node):
    def __init__(self):
        super().init__("Comand publisher (client-server)")
        self.publisher_ = self.create_publisher(string.String, "commands topic client-server", 10)

    def publish_on_event(self,event_data): 
        msg = string.String()
        msg.data = event_data
        self.publisher_.publish(msg)

class Subscriber(Node):
    def __init__(self):
        super().init__("Comand subscriber (client-server)")
        self.subscription = self.create_subscription(string.String, "commands topic server-client", self.listener_callback, 10)

    def listener_callback(self, msg):
        print(f"Received: {msg.data}")

class Command_client(ConnectionBase):
    def __init__(self):
        super().__init__("client","communication_port")

    def gather_message_info(self):
        return input("Mensaje: ")
    
    def manage_received_info(self,info):
        print(f"Received: {info}")
