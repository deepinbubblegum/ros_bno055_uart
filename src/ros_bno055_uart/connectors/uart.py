import sys
import rospy
import serial
from connectors.Connector import Connector

class UART(Connector):
    CONNECTIONTYPE_UART = 'uart'
    def __init__(self, node, baudrate, port, timeout):
        self.baudrate = baudrate
        self.port = port
        self.timeout = timeout
        self.serialConnection = None

    def connect(self):
        rospy.loginfo("Opening serial port: {}".format(self.port))
        try:
            self.serialConnection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except serial.serialutil.SerialException:
            rospy.loginfo("Unable to connect to IMU at port {}".format(self.port))
            rospy.loginfo("Check to make sure your device is connected")
            sys.exit(1)
    
    def read(self, numberOfBytes):
        return self.serialConnection.read(numberOfBytes)

    def write(self, data: bytearray):
        self.serialConnection.write(data)