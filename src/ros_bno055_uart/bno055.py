#!/usr/bin/env python3
import sys
import threading
from connectors.i2c import I2C
from connectors.uart import UART
from error_handling.exceptions import BusOverRunException
from sensor.SensorService import SensorService
import rospy
from params.NodeParameters import NodeParameters

class BNO055Node:
    sensor = None
    param = None
    def __init__(self):
        # init node
        rospy.init_node('BNO055_Node', anonymous=False)

    def setup(self):
        self.param = NodeParameters(self)

        # Get connector according to configured sensor connection type:
        if self.param.connection_type == UART.CONNECTIONTYPE_UART:
            connector = UART(
                self,
                self.param.uart_baudrate,
                self.param.uart_port,
                self.param.uart_timeout
            )
        elif self.param.connection_type == I2C.CONNECTIONTYPE_I2C:
            # TODO implement IC2 integration
            raise NotImplementedError('I2C not yet implemented')
        else:
            raise NotImplementedError('Unsupported connection type: '
                                      + str(self.param.connection_type))

        # Connect to BNO055 device: 
        connector.connect()
        
        # Instantiate the sensor Service API:
        self.sensor = SensorService(self, connector, self.param)

        # configure imu
        self.sensor.configure()

def main():
    try:
        node = BNO055Node()
        node.setup()
        # Create lock object to prevent overlapping data queries
        lock = threading.Lock()
        def read_data(none):
            """Periodic data_query_timer executions to retrieve sensor IMU data."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                rospy.logwarn('Message communication in progress - skipping query cycle')
                return
            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_sensor_data()
            except BusOverRunException:
                # data not available yet, wait for next cycle | see #5
                return
            except Exception as e:  # noqa: B902
                rospy.logwarn('Receiving sensor data failed with %s:"%s"'
                                    % (type(e).__name__, e))
            finally:
                lock.release()

        def log_calibration_status(none):
            """Periodic logging of calibration data (quality indicators)."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                rospy.logwarn('Message communication in progress - skipping query cycle')
                # traceback.print_exc()
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_calib_status()
            except Exception as e:  # noqa: B902
                rospy.logwarn('Receiving calibration status failed with %s:"%s"'
                                       % (type(e).__name__, e))
                # traceback.print_exc()
            finally:
                lock.release()

        f = 1.0 / float(node.param.data_query_frequency)
        data_query_timer = rospy.Timer(rospy.Duration(f), read_data)
        
        # start regular calibration status logging
        f = 1.0 / float(node.param.calib_status_frequency)
        status_timer = rospy.Timer(rospy.Duration(f), log_calibration_status)

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Ctrl+C received - exiting...')
        sys.exit(0)
    finally:
        rospy.loginfo('ROS node shutdown')

if __name__ == '__main__':
    main()