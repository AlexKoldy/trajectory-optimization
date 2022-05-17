from threading import Thread
from multiprocessing import SimpleQueue as Queue
import socket
import select
import traceback

# TODO: Fix paths
from .comms_protocol import Message, CommsProtocol, Decoder
from .client import TestClass


class Server(Thread):
    def __init__(self, ip, port: int):
        Thread.__init__(self, target=self.run)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.msg_queue = Queue()  # Queue for message

        # Setup TCP Socket
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((ip, port))
        self.socket.listen(5)
        self.daemon = True
        self.start()

    def run(self):
        """Run the socket server.
        1. Receive message over socket
        2. Convert to Message class
        3. Put message into Queue
        4. Shutdown client socket
        """
        while True:
            try:
                print("Waiting for socket connection...")
                client, addr = self.socket.accept()
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
                    print("Ready to read socket data. Reading...")
                    msg = self.receive_message(client)
                    print("Message from socket: ---------------------------")
                    print(msg.type, msg.data)
                    print("-----------------------------")
                    print("Adding message to queue...")
                    self.msg_queue.put(msg)

            except socket.error as error:  # , msg:
                print("Socket error! %s" % error.strerror)
                pass
            except Exception as e:
                print(e)
                traceback.print_exc()
                break

        # Shutdown the socket
        try:
            print("Shutting down socket...")
            self.socket.shutdown(socket.SHUT_RDWR)
        except:
            print("Force closed socket")
            self.socket.close()
            self.run()

    def receive_message(self, sock):
        """Receive full message even if it's larger than buffer size (as a string)
        Decode into Message class and return
        Args:
            sock ([type]): The socket to receive the bytes on
        Returns:
            [type]: [description]
        """
        BUFF_SIZE = 4096  # 4 KiB
        data = b""
        while True:

            part = sock.recv(BUFF_SIZE)
            data += part
            if len(part) <= 0:
                # either 0 or end of data
                break

        return Decoder.decode(data)


# if __name__ == "__main__":
#     """Testing the Server process and queue"""
#     try:
#         s = Server(
#             CommsProtocol.SERVER, CommsProtocol.PORT
#         )  # Instantiate server class (which starts the process)
#         while True:
#             try:
#                 """Print whatever messages are in the queue"""
#                 msg = s.msg_queue.get()
#                 print("Message from queue:------------------")
#                 print(msg.type, msg.data)
#                 print("--------------------")

#             except:
#                 traceback.print_exc()
#                 break
#     except KeyboardInterrupt:
#         print("Exiting...")
#         s.terminate()
