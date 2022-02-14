from connectors.Connector import Connector
import rospy

class I2C(Connector):
    CONNECTIONTYPE_I2C = 'i2c'

    def __init__(self):
        # Initialize parent
        super().__init__()
        
    def connect(self):
        raise NotImplementedError('I2C not yet implemented')

    def read(self, numberOfBytes):
        raise NotImplementedError('I2C not yet implemented')

    def write(self, data: bytearray):
        raise NotImplementedError('I2C not yet implemented')