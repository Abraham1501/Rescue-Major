#usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclp import string
from base_socket import ConnectionBase
import threading

class Publisher(Node):
    def __init__(self, socket_class):
        super().init__("Comand publisher (server-client)")
        self.sok = socket_class
        self.publisher_ = self.create_publisher(string.String, "commands topic server-client", 10)

    def publish_on_event(self): 
        msg = string.String()
        msg.data = self.sok.gather_message_info()
        if msg.data == "exit":
            print("Exiting...")
            rclpy.shutdown()
        self.publisher_.publish(msg)

class Subscriber(Node):
    def __init__(self,socket_class):
        super().init__("Comand subscriber server-client")
        self.sok = socket_class
        self.subscription = self.create_subscription(string.String, "commands topic client-server", self.listener_callback, 10)

    def listener_callback(self, msg):
        self.sok.manage_received_info(msg.data)

class Command_server(ConnectionBase):
    def __init__(self):
        super().__init__("server","communication_port")

    def gather_message_info(self):
        return input("Mensaje: ")
    
    def manage_received_info(self,info):
        print(f"Received: {info}")

if __name__ == "main":
    rclpy.init()
    com_server = Command_server()
    pub = Publisher(com_server)
    sub = Subscriber(com_server)


#--------------------------

class Subscriber(Node):
    def __init__(self, command_server):
        super().__init__("command_subscriber")
        self.subscription = self.create_subscription(
            String, "commands_topic", self.listener_callback, 10
        )
        self.command_server = command_server  
        self.event = threading.Event()  

    def listener_callback(self, msg):
        print(f"Received: {msg.data}")
        self.command_server.message_data = msg.data  # Almacenar mensaje recibido
        self.event.set()  # Desbloquear el método gather_message_info()

class CommandServer:
    def __init__(self):
        self.message_data = None  # Almacenar la información recibida

    def gather_message_info(self, subscriber):
        print("Esperando mensaje del subscriber...")
        subscriber.event.wait()  # Esperar hasta que el subscriber reciba un mensaje
        subscriber.event.clear()  # Resetear el evento para futuras ejecuciones
        return self.message_data  # Retornar el mensaje recibido

def main(args=None):
    rclpy.init(args=args)

    command_server = CommandServer()
    subscriber = Subscriber(command_server)

    # Simular proceso que espera información del subscriber
    message = command_server.gather_message_info(subscriber)
    print(f"Información obtenida del subscriber: {message}")

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
