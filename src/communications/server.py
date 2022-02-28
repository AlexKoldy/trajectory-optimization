import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

from threading import Thread
from multiprocessing import SimpleQueue as Queue
import socket
import select
import traceback
import pickle

# from src.utilities.history import History

from src.communications.comms_protocol import Message, CommsProtocol, Decoder, Encoder
from src.communications.client import TestClass

HEADERSIZE = 10


class Server(Thread):
    def __init__(self, ip, port: int):
        Thread.__init__(self, target=self.run)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.msg_queue = Queue()  # Queue for message

        # Setup TCP Socket
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((ip, port))
        self.sock.listen(5)
        self.daemon = True
        self.start()

        self.state = 0

    def run(self):
        """Run the socket server.
        1. Receive message over socket
        2. Convert to Message class
        3. Put message into Queue
        4. Shutdown client socket
        """
        self.state = 0
        client, addr = self.sock.accept()
        while True:
            try:
                if self.state == 0:
                    # Wait for message from client
                    print("WAITING FOR MESSAGE FROM CLIENT")
                    if self.msg_queue.empty():
                        print("Waiting for socket connection...")

                        # client.settimeout(5)
                        ready = select.select(
                            [
                                client,
                            ],
                            [],
                            [],
                            2,
                        )
                        # print("Connection started.")
                        if ready[0]:
                            print("Ready to read socket data. Reading...")
                            msg = self.receive_message(client)
                            print(f"Message from socket: {msg.type, msg.data}")
                            print("Adding message to queue...")
                            self.msg_queue.put(msg)
                            self.state = 1
                elif self.state == 1:
                    # Do task based off of client message
                    print("COMPLETING A TASK")
                    self.state = 2
                elif self.state == 2:
                    # Reply to client
                    print("SENDING RESPONSE TO CLIENT")
                    # client, addr = self.sock.accept()
                    print("Test1")
                    # client.settimeout(5)
                    print("Test2")
                    ready = select.select(
                        [
                            client,
                        ],
                        [],
                        [],
                        2,
                    )
                    print("Connection started.")
                    if ready[0]:
                        self.send_message(
                            msg_type="initalize state", data="test", sock=client
                        )
                        self.state = 0

                "If the message sent "
                """
                if ready[
                    0
                ]:  # not sure if a new run needed to be written or if it could fit here
                    d = history.x_history
                    msg = pickle.dumps(d)
                    msg = bytes(f"{len(msg):<{HEADERSIZE}}", "utf-8") + msg
                    # print(msg)
                    client.send(msg)
                """

            except socket.error as error:
                print("Socket error! %s" % error.strerror)
                traceback.print_exc()
                break
            except Exception as e:
                print(e)
                traceback.print_exc()
                break

        # Shutdown the socket
        try:
            print("Shutting down socket...")
            client.shutdown(socket.SHUT_RDWR)
        except:
            print("Force closed socket")
            client.close()
            self.run()

    def receive_message(self, sock):
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
                part = sock.recv(BUFF_SIZE)
                data += part
            except Exception as e:
                print(e)
                break
        return Decoder.decode(data)

    def send_message(self, msg_type, data, sock):
        """
        Send a message over the socket

        Args:
            msg_type ([type]): the type of message being sent (must come from the CommsProtocol class)
            data ([type]): the class or data to be sent along with the message
        """
        try:
            send_data = Encoder.encode(msg_type, data)
            sock.send(send_data)
            print("Send: ", send_data)

        except Exception as e:
            print(e)
            traceback.print_exc()


"""
    def echo_message(self, data):
        d = history.x_history
        msg = pickle.dumps(d)
        msg = bytes(f"{len(msg):<{HEADERSIZE}}", 'utf-8')+msg
        #print(msg)
        client.send(msg)
"""

if __name__ == "__main__":
    """Testing the Server process and queue"""
    try:
        s = Server(
            CommsProtocol.SERVER, CommsProtocol.PORT
        )  # Instantiate server class (which starts the process)
        while True:
            """
            try:
                #Print whatever messages are in the queue
                msg = s.msg_queue[0]
                print("Message from queue:------------------")
                print(msg.type, msg.data)
                print("--------------------")

            except:
                traceback.print_exc()
                break
            """
    except KeyboardInterrupt:
        print("Exiting...")
        s.terminate()
