from base_socket import ConnectionBase

class Server(ConnectionBase):
    def __init__(self, mode, port):
        super().__init__(mode, port)

    def bind_socket_ifneeded(self):
        self.socket.bind((self.host, self.port))

def obtain_info():
    return input("Mensaje:  ")

def print_info(info):
    print(f"ans {info}")

a = Server("server", "communication_port")
a.manage_info = print_info
a.gather_info = obtain_info

a.start()