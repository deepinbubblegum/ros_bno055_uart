import imp
import sys
import struct
import json
import rospy
from math import sqrt
from time import sleep
import registers
from connectors.Connector import Connector
from params.NodeParameters import NodeParameters
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String
from std_srvs.srv import Trigger

class SensorService:
    """Provide an interface for accessing the sensor's features & data."""

    def __init__(self, node, connector: Connector, param: NodeParameters):
        self.con = connector
        self.param = param
        
        prefix = self.param.ros_topic_prefix

        # create topic publishers:
        self.pub_imu_raw = rospy.Publisher(prefix + 'imu_raw' , Imu, queue_size=10)
        self.pub_imu = rospy.Publisher(prefix + 'imu' , Imu, queue_size=10)
        self.pub_mag = rospy.Publisher(prefix + 'mag' , MagneticField, queue_size=10)
        self.pub_temp = rospy.Publisher(prefix + 'temp' , Temperature, queue_size=10)
        self.pub_calib_status = rospy.Publisher(prefix + 'calib_status' , String, queue_size=10)
        self.srv = rospy.Service(prefix + 'calibration_request', Trigger,  self.calibration_request_callback)

    def configure(self):
        """Configure the IMU sensor hardware."""
        rospy.loginfo('Configuring device...')
        try:
          data = self.con.receive(registers.BNO055_CHIP_ID_ADDR, 1)
          if data[0] != registers.BNO055_ID:
              raise IOError('Device ID=%s is incorrect' % data)
        except Exception as e:
            # This is the first communication - exit if it does not work
            rospy.logerr('Communication error: %s' % e)
            rospy.logerr('Shutting down ROS node...')
            sys.exit(1)

        # IMU connected => apply IMU Configuration:
        if not(self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            rospy.logwarn('Unable to set IMU into config mode.')

        if not(self.con.transmit(registers.BNO055_PWR_MODE_ADDR, 1, bytes([registers.POWER_MODE_NORMAL]))):
            rospy.logwarn('Unable to set IMU normal power mode.')

        if not (self.con.transmit(registers.BNO055_PAGE_ID_ADDR, 1, bytes([0x00]))):
            rospy.logwarn('Unable to set IMU register page 0.')

        if not (self.con.transmit(registers.BNO055_SYS_TRIGGER_ADDR, 1, bytes([0x00]))):
            rospy.logwarn('Unable to start IMU.')

        if not (self.con.transmit(registers.BNO055_UNIT_SEL_ADDR, 1, bytes([0x83]))):
            rospy.logwarn('Unable to set IMU units.')

        # The sensor placement configuration (Axis remapping) defines the
        # position and orientation of the sensor mount.
        # See also Bosch BNO055 datasheet section Axis Remap
        mount_positions = {
            'P0': bytes(b'\x21\x04'),
            'P1': bytes(b'\x24\x00'),
            'P2': bytes(b'\x24\x06'),
            'P3': bytes(b'\x21\x02'),
            'P4': bytes(b'\x24\x03'),
            'P5': bytes(b'\x21\x02'),
            'P6': bytes(b'\x21\x07'),
            'P7': bytes(b'\x24\x05')
        }
        if not (self.con.transmit(registers.BNO055_AXIS_MAP_CONFIG_ADDR, 2,
                                  mount_positions[self.param.placement_axis_remap])):
            rospy.logwarn('Unable to set sensor placement configuration.')

        # Show the current sensor offsets
        rospy.logerr('Current sensor offsets:')
        self.print_calib_data()
        if self.param.set_offsets:
            configured_offsets = \
                self.set_calib_offsets(
                    self.param.offset_acc,
                    self.param.offset_mag,
                    self.param.offset_gyr,
                    self.param.radius_mag,
                    self.param.radius_acc)
            if configured_offsets:
                rospy.logerr('Successfully configured sensor offsets to:')
                self.print_calib_data()
            else:
                rospy.logwarn('setting offsets failed')

        # Set Device to NDOF mode
        # data fusion for gyroscope, acceleration sensor and magnetometer enabled
        # absolute orientation
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_NDOF]))):
            rospy.logwarn('Unable to set IMU operation mode into operation mode.')

        rospy.loginfo('Bosch BNO055 IMU configuration complete.')

    def get_sensor_data(self):
        """Read IMU data from the sensor, parse and publish."""
        # Initialize ROS msgs
        imu_raw_msg = Imu()
        imu_msg = Imu()
        mag_msg = MagneticField()
        temp_msg = Temperature()

        # read from sensor
        buf = self.con.receive(registers.BNO055_ACCEL_DATA_X_LSB_ADDR, 45)
        # Publish raw data
        imu_raw_msg.header.stamp = rospy.Time.now()
        imu_raw_msg.header.frame_id = self.param.frame_id
        # TODO: make this an option to publish?
        # print(self.param.variance_orientation)
        imu_raw_msg.orientation_covariance = [
            self.param.variance_orientation[0], 0.0, 0.0,
            0.0, self.param.variance_orientation[1], 0.0,
            0.0, 0.0, self.param.variance_orientation[2]
        ]

        imu_raw_msg.linear_acceleration.x = self.unpackBytesToFloat(buf[0], buf[1]) / self.param.acc_factor
        imu_raw_msg.linear_acceleration.y = self.unpackBytesToFloat(buf[2], buf[3]) / self.param.acc_factor
        imu_raw_msg.linear_acceleration.z = self.unpackBytesToFloat(buf[4], buf[5]) / self.param.acc_factor
        imu_raw_msg.linear_acceleration_covariance = [
            self.param.variance_acc[0], 0.0, 0.0,
            0.0, self.param.variance_acc[1], 0.0,
            0.0, 0.0, self.param.variance_acc[2]
        ]
        imu_raw_msg.angular_velocity.x = self.unpackBytesToFloat(buf[12], buf[13]) / self.param.gyr_factor
        imu_raw_msg.angular_velocity.y = self.unpackBytesToFloat(buf[14], buf[15]) / self.param.gyr_factor
        imu_raw_msg.angular_velocity.z = self.unpackBytesToFloat(buf[16], buf[17]) / self.param.gyr_factor
        imu_raw_msg.angular_velocity_covariance = [
            self.param.variance_angular_vel[0], 0.0, 0.0,
            0.0, self.param.variance_angular_vel[1], 0.0,
            0.0, 0.0, self.param.variance_angular_vel[2]
        ]
        self.pub_imu_raw.publish(imu_raw_msg)

        # TODO: make this an option to publish?
        # Publish filtered data
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = self.param.frame_id

        q = Quaternion()
        q.w = self.unpackBytesToFloat(buf[24], buf[25])
        q.x = self.unpackBytesToFloat(buf[26], buf[27])
        q.y = self.unpackBytesToFloat(buf[28], buf[29])
        q.z = self.unpackBytesToFloat(buf[30], buf[31])

        # TODO(flynneva): replace with standard normalize() function
        # normalize
        norm = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        imu_msg.orientation.x = q.x / norm
        imu_msg.orientation.y = q.y / norm
        imu_msg.orientation.z = q.z / norm
        imu_msg.orientation.w = q.w / norm

        imu_msg.orientation_covariance = imu_raw_msg.orientation_covariance

        imu_msg.linear_acceleration.x = \
            self.unpackBytesToFloat(buf[32], buf[33]) / self.param.acc_factor
        imu_msg.linear_acceleration.y = \
            self.unpackBytesToFloat(buf[34], buf[35]) / self.param.acc_factor
        imu_msg.linear_acceleration.z = \
            self.unpackBytesToFloat(buf[36], buf[37]) / self.param.acc_factor
        imu_msg.linear_acceleration_covariance = imu_raw_msg.linear_acceleration_covariance
        imu_msg.angular_velocity.x = \
            self.unpackBytesToFloat(buf[12], buf[13]) / self.param.gyr_factor
        imu_msg.angular_velocity.y = \
            self.unpackBytesToFloat(buf[14], buf[15]) / self.param.gyr_factor
        imu_msg.angular_velocity.z = \
            self.unpackBytesToFloat(buf[16], buf[17]) / self.param.gyr_factor
        imu_msg.angular_velocity_covariance = imu_raw_msg.angular_velocity_covariance
        self.pub_imu.publish(imu_msg)

        # Publish magnetometer data
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.header.frame_id = self.param.frame_id
        # mag_msg.header.seq = seq
        mag_msg.magnetic_field.x = \
            self.unpackBytesToFloat(buf[6], buf[7]) / self.param.mag_factor
        mag_msg.magnetic_field.y = \
            self.unpackBytesToFloat(buf[8], buf[9]) / self.param.mag_factor
        mag_msg.magnetic_field.z = \
            self.unpackBytesToFloat(buf[10], buf[11]) / self.param.mag_factor
        mag_msg.magnetic_field_covariance = [
            self.param.variance_mag[0], 0.0, 0.0,
            0.0, self.param.variance_mag[1], 0.0,
            0.0, 0.0, self.param.variance_mag[2]
        ]
        self.pub_mag.publish(mag_msg)

        # Publish temperature
        temp_msg.header.stamp = rospy.Time.now()
        temp_msg.header.frame_id = self.param.frame_id
        # temp_msg.header.seq = seq
        temp_msg.temperature = float(buf[44])
        self.pub_temp.publish(temp_msg)
    
    def get_calib_status(self):
        """
        Read calibration status for sys/gyro/acc/mag.
        Quality scale: 0 = bad, 3 = best
        """
        calib_status = self.con.receive(registers.BNO055_CALIB_STAT_ADDR, 1)
        sys = (calib_status[0] >> 6) & 0x03
        gyro = (calib_status[0] >> 4) & 0x03
        accel = (calib_status[0] >> 2) & 0x03
        mag = calib_status[0] & 0x03

        # Create dictionary (map) and convert it to JSON string:
        calib_status_dict = {'sys': sys, 'gyro': gyro, 'accel': accel, 'mag': mag}
        calib_status_str = String()
        calib_status_str.data = json.dumps(calib_status_dict)

        # Publish via ROS topic:
        self.pub_calib_status.publish(calib_status_str)
    
    def get_calib_data(self):
        """Read all calibration data."""

        accel_offset_read = self.con.receive(registers.ACCEL_OFFSET_X_LSB_ADDR, 6)
        accel_offset_read_x = (accel_offset_read[1] << 8) | accel_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        accel_offset_read_y = (accel_offset_read[3] << 8) | accel_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        accel_offset_read_z = (accel_offset_read[5] << 8) | accel_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        accel_radius_read = self.con.receive(registers.ACCEL_RADIUS_LSB_ADDR, 2)
        accel_radius_read_value = (accel_radius_read[1] << 8) | accel_radius_read[0]

        mag_offset_read = self.con.receive(registers.MAG_OFFSET_X_LSB_ADDR, 6)
        mag_offset_read_x = (mag_offset_read[1] << 8) | mag_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        mag_offset_read_y = (mag_offset_read[3] << 8) | mag_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        mag_offset_read_z = (mag_offset_read[5] << 8) | mag_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        mag_radius_read = self.con.receive(registers.MAG_RADIUS_LSB_ADDR, 2)
        mag_radius_read_value = (mag_radius_read[1] << 8) | mag_radius_read[0]

        gyro_offset_read = self.con.receive(registers.GYRO_OFFSET_X_LSB_ADDR, 6)
        gyro_offset_read_x = (gyro_offset_read[1] << 8) | gyro_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        gyro_offset_read_y = (gyro_offset_read[3] << 8) | gyro_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        gyro_offset_read_z = (gyro_offset_read[5] << 8) | gyro_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        calib_data = {'accel_offset': {'x': accel_offset_read_x, 'y': accel_offset_read_y, 'z': accel_offset_read_z}, 'accel_radius': accel_radius_read_value,
                      'mag_offset': {'x': mag_offset_read_x, 'y': mag_offset_read_y, 'z': mag_offset_read_z}, 'mag_radius': mag_radius_read_value,
                      'gyro_offset': {'x': gyro_offset_read_x, 'y': gyro_offset_read_y, 'z': gyro_offset_read_z}}

        return calib_data

    def print_calib_data(self):
        """Read all calibration data and print to screen."""
        calib_data = self.get_calib_data()
        rospy.loginfo(
            '\tAccel offsets (x y z): %d %d %d' % (
                calib_data['accel_offset']['x'],
                calib_data['accel_offset']['y'],
                calib_data['accel_offset']['z']))

        rospy.loginfo(
            '\tAccel radius: %d' % (
                calib_data['accel_radius'],
            )
        )

        rospy.loginfo(
            '\tMag offsets (x y z): %d %d %d' % (
                calib_data['mag_offset']['x'],
                calib_data['mag_offset']['y'],
                calib_data['mag_offset']['z']))

        rospy.loginfo(
            '\tMag radius: %d' % (
                calib_data['mag_radius'],
            )
        )

        rospy.loginfo(
            '\tGyro offsets (x y z): %d %d %d' % (
                calib_data['gyro_offset']['x'],
                calib_data['gyro_offset']['y'],
                calib_data['gyro_offset']['z']))

    def set_calib_offsets(self, acc_offset, mag_offset, gyr_offset, mag_radius, acc_radius):
        """
        Write calibration data (define as 16 bit signed hex).
        :param acc_offset:
        :param mag_offset:
        :param gyr_offset:
        :param mag_radius:
        :param acc_radius:
        """
        # Must switch to config mode to write out
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            rospy.logerr('Unable to set IMU into config mode')
        sleep(0.025)

        # Seems to only work when writing 1 register at a time
        try:
            self.con.transmit(registers.ACCEL_OFFSET_X_LSB_ADDR, 1, bytes([acc_offset[0] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_X_MSB_ADDR, 1, bytes([(acc_offset[0] >> 8) & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Y_LSB_ADDR, 1, bytes([acc_offset[1] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Y_MSB_ADDR, 1, bytes([(acc_offset[1] >> 8) & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Z_LSB_ADDR, 1, bytes([acc_offset[2] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Z_MSB_ADDR, 1, bytes([(acc_offset[2] >> 8) & 0xFF]))

            self.con.transmit(registers.ACCEL_RADIUS_LSB_ADDR, 1, bytes([acc_radius & 0xFF]))
            self.con.transmit(registers.ACCEL_RADIUS_MSB_ADDR, 1, bytes([(acc_radius >> 8) & 0xFF]))

            self.con.transmit(registers.MAG_OFFSET_X_LSB_ADDR, 1, bytes([mag_offset[0] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_X_MSB_ADDR, 1, bytes([(mag_offset[0] >> 8) & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Y_LSB_ADDR, 1, bytes([mag_offset[1] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Y_MSB_ADDR, 1, bytes([(mag_offset[1] >> 8) & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Z_LSB_ADDR, 1, bytes([mag_offset[2] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Z_MSB_ADDR, 1, bytes([(mag_offset[2] >> 8) & 0xFF]))

            self.con.transmit(registers.MAG_RADIUS_LSB_ADDR, 1, bytes([mag_radius & 0xFF]))
            self.con.transmit(registers.MAG_RADIUS_MSB_ADDR, 1, bytes([(mag_radius >> 8) & 0xFF]))

            self.con.transmit(registers.GYRO_OFFSET_X_LSB_ADDR, 1, bytes([gyr_offset[0] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_X_MSB_ADDR, 1, bytes([(gyr_offset[0] >> 8) & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Y_LSB_ADDR, 1, bytes([gyr_offset[1] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Y_MSB_ADDR, 1, bytes([(gyr_offset[1] >> 8) & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Z_LSB_ADDR, 1, bytes([gyr_offset[2] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Z_MSB_ADDR, 1, bytes([(gyr_offset[2] >> 8) & 0xFF]))

            return True
        except Exception:  # noqa: B902
            return False

    def calibration_request_callback(self, request, response):
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            rospy.logwarn('Unable to set IMU into config mode.')
        sleep(0.025)
        calib_data = self.get_calib_data()
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_NDOF]))):
            rospy.logwarn('Unable to set IMU operation mode into operation mode.')
        response.success = True
        response.message = str(calib_data)
        return response

    def unpackBytesToFloat(self, start, end):
        return float(struct.unpack('h', struct.pack('BB', start, end))[0])