import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

from threading import Thread
from multiprocessing import SimpleQueue as Queue
from multiprocessing import Process
import socket
import traceback
import pickle

from src.communications.comms_protocol import Message, CommsProtocol, Decoder, Encoder

# HEADERSIZE = 10


class Client(Thread):
    def __init__(self):
        Thread.__init__(self, target=self.run)

        self.state = 0
        self.msg_queue = Queue()
        self.gui_msg_queue = Queue()

        # Setup TCP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(CommsProtocol.ADDR)
        self.daemon = True
        self.start()

    def run(self):
        self.state = 0
        while True:
            try:
                if self.state == 0:
                    # Wait for message in GUI message queue and send message to server
                    print("SENDING MESSAGE TO SERVER")
                    if not self.gui_msg_queue.empty():
                        msg = self.gui_msg_queue.get()
                        self.send_message(msg_type=msg.type, data=msg.data)
                        self.state = 1
                elif self.state == 1:
                    # Wait for response from server
                    print("WAITING FOR SERVER'S RESPONSE")
                    if self.msg_queue.empty():
                        print("Waiting for socket connection...")
                        # server, addr = self.sock.accept()
                        # server.settimeout(5)
                        # ready = select.select(
                        #     [
                        #         server,
                        #     ],
                        #     [],
                        #     [],
                        #     2,
                        # )
                        # print("Connection started.")
                        # if ready[0]:
                        print("Ready to read socket data. Reading...")
                        msg = self.receive_message()
                        print(f"Message from socket: {msg.type, msg.data}")
                        print("Adding message to queue...")
                        self.msg_queue.put(msg)
                        self.state = 2
                elif self.state == 2:
                    # Do task based off of server response
                    print("COMPLETING A TASK")
                    self.state == 0

            except Exception as e:
                print(e)
                pass

    def receive_message(self):
        """
        Receive full message even if it's larger than buffer size (as a string)
        Decode into Message class and return

        Args:
            sock ([type]): The socket to receive the bytes on

        Returns:
            [type]: [description]
        """
        BUFF_SIZE = 4096  # 4 KiB
        data = b""
        while True:
            try:
                part = self.sock.recv(BUFF_SIZE)
                data += part
            except Exception as e:
                print(e)
                break
        return Decoder.decode(data)

    def send_message(self, msg_type, data):
        """
        Send a message over the socket

        Args:
            msg_type ([type]): the type of message being sent (must come from the CommsProtocol class)
            data ([type]): the class or data to be sent along with the message
        """
        try:
            send_data = Encoder.encode(msg_type, data)
            self.sock.send(send_data)
            print("Send: ", send_data)

        except Exception as e:
            print(e)
            traceback.print_exc()


"""
    def get_data(self):
        "works with a server that echos data back that the server recieves"
        # client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # client.connect(CommsProtocol.ADDR)
        # client.sendall(b'Hello, world')
        # data = client.pickle.loads(recv(1024))
        # print('Received', data)
        full_msg = b""
        new_msg = True
        while True:
            msg = self.sock.recv(16)
            if new_msg:
                print("new msg len:", msg[:HEADERSIZE])
                msglen = int(msg[:HEADERSIZE])
                new_msg = False

            print(f"full message length: {msglen}")

            full_msg += msg

            print(len(full_msg))

            if len(full_msg) - HEADERSIZE == msglen:
                print("full msg recvd")
                print(full_msg[HEADERSIZE:])
                print(pickle.loads(full_msg[HEADERSIZE:]))
                new_msg = True
                full_msg = b""
"""


class TestClass:
    def __init__(self):
        self.val = 20


if __name__ == "__main__":
    """Test the use of the Client() class, send execute message and TestClass"""
    from src.robot.state import State

    q = "hi"
    c = Client()
    c.gui_msg_queue.put(Message(CommsProtocol.types["initialize state"], q))
    # c.get_data()
    import time

    while True:
        time.sleep(1)
