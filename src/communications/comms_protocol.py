import pickle


class CommsProtocol:
    types = {
        "initialize state": "initialize state",
    }
    som = "%%%"  # Start of message key
    sod = "$$$"  # Start of data key
    eom = "###"  # End of message key
    SERVER = "127.0.0.1"
    PORT = 5050
    ADDR = (SERVER, PORT)
    FORMAT = "utf-8"


class Message:
    def __init__(self, type="None", data=None):
        self.type = type
        self.data = data


class Decoder:
    @staticmethod
    def decode(msg: bytes):
        """Convert the message string into it's message type and data parts

        Args:
            msg (Message): [description]
        """
        # Split by the som, sod, eod
        msg_type, data = str(msg, "utf-8").split(CommsProtocol.sod)
        msg_type = msg_type.replace(CommsProtocol.som, "")  # Remove som
        # data = data.replace("\\",'')
        data = data.replace(CommsProtocol.eom, "")  # Remove eom
        return Message(msg_type, data)


class Encoder:
    @staticmethod
    def encode(msg_type, data):
        """Encode the message into a string using som, sod, eod from CommsProtocol

        Args:
            type ([type]): [description]
            data ([type]): [description]
        """
        string = CommsProtocol.som  # Add start of msg (som)
        string += msg_type  # Add type string
        string += CommsProtocol.sod  # Add start of data (sod)
        # Marshmellow serialize class
        string += data  # Add data
        string += CommsProtocol.eom  # Add end of msg (eom)
        return bytes(string, "utf-8")
