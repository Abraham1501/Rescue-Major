from base_socket import ConnectionBase
import threading
import time 

class CLient(ConnectionBase):
    def __init__(self, mode, port):
        super().__init__(mode, port)

def obtain_info():
    return input("Mensaje:  ")

def print_info(info):
    print(f"ans {info}")

b = CLient("client", "communication_port")
b.manage_info = print_info
b.gather_info = obtain_info

b.start()

"""

    def start(self):
        conections = 0
        while not self.end_instance:  
            try:
                self.logger.info(f"Starting {self.mode}, attempt {conections}")
                conections += 1
                self.connect()  
                t1 = threading.Thread(target=self.read)
                t2 = threading.Thread(target=self.send)
                t1.start()
                t2.start()
                t1.join()
                t2.join()
            except Exception as e:
                self.logger.error(f"Error en los hilos: {e}")
            finally:
                self.logger.warning("Threads finished, closing socket")  
                self.socket.close()  
                time.sleep(2)  
                if ("contnue: ").strip().lower() == 'n':
                    self.kill()
"""