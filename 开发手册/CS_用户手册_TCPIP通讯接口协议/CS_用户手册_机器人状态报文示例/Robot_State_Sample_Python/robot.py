import struct
import socket
import select
from RobotData import *
import time
    
DEFAULT_TIMEOUT = 10.0

ROBOT_STATE_TYPE = 16
ROBOT_EXCEPTION = 20

class Robot():
    def __init__(self, excel, sheet, robot_exception_hook = None) -> None:
        self.__data_config = RobotDataConfig.get_config(excel, sheet)
        self.exception_hook = robot_exception_hook

    def connect(self, ip, port = 30001):
        try:
            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.__sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.__sock.settimeout(DEFAULT_TIMEOUT)
            self.hostname = ip
            self.port = port
            self.__sock.connect((self.hostname, self.port))
            self.__buf = bytes()
        except (socket.timeout, socket.error):
            self.__sock = None
            raise
    
    def disconnect(self):
        self.__sock.close()
        self.__sock = None

    def get_data(self):
        return self.__recv()
    
    def send(self, script):
        return self.__sock.send(script)

    def __recv(self):
        try:
            self.__recv_to_buffer(DEFAULT_TIMEOUT)
        except:
            return None
        while len(self.__buf) > 5:
            head = RobotHeader.unpack(self.__buf)
            if head.size <= len(self.__buf):
                if head.type == ROBOT_STATE_TYPE:
                    data = RobotData.unpack(self.__buf, self.__data_config)
                    self.__buf = self.__buf[head.size :]
                    return data
                elif head.type == ROBOT_EXCEPTION:
                    exception = RobotException.unpack_exception(self.__buf)
                    if self.exception_hook is not None and exception is not None:
                        self.exception_hook(exception)
                    self.__buf = self.__buf[head.size :]
                    return None
                else:
                    self.__buf = self.__buf[head.size :]
                    continue
            else:
                break
        return None

    def __recv_to_buffer(self, timeout):
        readable, _, xlist = select.select([self.__sock], [], [self.__sock], timeout)
        if len(readable):
            more = self.__sock.recv(4096)
            # When the controller stops while the script is running
            if len(more) == 0:
                print('received 0 bytes from Controller')
                return None
            
            self.__buf = self.__buf + more
            return True
        
        if (len(xlist) or len(readable) == 0) and timeout != 0: # Effectively a timeout of timeout seconds
            print ("no data received within timeout")
            return None

        return False
    