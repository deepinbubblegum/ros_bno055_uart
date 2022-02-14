import rospy
from connectors.uart import UART
import registers

class NodeParameters:
    def __init__(self, node):
        # Get node name
        self.node_name = rospy.get_name()
        rospy.loginfo('Initializing parameters')
        self.ros_topic_prefix = rospy.get_param(self.node_name + '/ros_topic_prefix', 'bno055/')
        self.connection_type = rospy.get_param(self.node_name + '/connection_type', UART.CONNECTIONTYPE_UART)
        self.uart_port = rospy.get_param(self.node_name + '/uart_port', '/dev/ttyUSB0')
        self.uart_baudrate = rospy.get_param(self.node_name + '/uart_baudrate', '115200')
        self.uart_timeout = rospy.get_param(self.node_name + '/uart_timeout', '0.1')
        self.data_query_frequency = rospy.get_param(self.node_name + '/data_query_frequency', '100.0')
        self.calib_status_frequency = rospy.get_param(self.node_name + '/calib_status_frequency', '0.1')
        self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'imu_link')
        self.operation_mode = rospy.get_param(self.node_name + '/operation_mode', '0x0C')
        self.placement_axis_remap = rospy.get_param(self.node_name + '/placement_axis_remap', 'P2')
        self.acc_factor = rospy.get_param(self.node_name + '/acc_factor', '100.0')
        self.mag_factor = rospy.get_param(self.node_name + '/mag_factor', '16000000.0')
        self.gyr_factor = rospy.get_param(self.node_name + '/gyr_factor', '900.0')
        self.set_offsets = rospy.get_param(self.node_name + '/set_offsets', 'false')
        self.offset_acc = rospy.get_param(self.node_name + '/offset_acc', registers.DEFAULT_OFFSET_ACC)
        self.offset_mag = rospy.get_param(self.node_name + '/offset_mag', registers.DEFAULT_OFFSET_MAG)
        self.offset_gyr = rospy.get_param(self.node_name + '/offset_gyr', registers.DEFAULT_OFFSET_GYR)
        self.radius_acc = rospy.get_param(self.node_name + '/radius_acc', registers.DEFAULT_RADIUS_ACC)
        self.radius_mag = rospy.get_param(self.node_name + '/radius_mag', registers.DEFAULT_RADIUS_MAG)
        self.variance_acc = self.Convert(rospy.get_param(self.node_name + '/variance_acc', registers.DEFAULT_VARIANCE_ACC))
        self.variance_angular_vel = self.Convert(rospy.get_param(self.node_name + '/variance_angular_vel', registers.DEFAULT_VARIANCE_ANGULAR_VEL))
        self.variance_orientation = self.Convert(rospy.get_param(self.node_name + '/variance_orientation', registers.DEFAULT_VARIANCE_ORIENTATION))
        self.variance_mag = self.Convert(rospy.get_param(self.node_name + '/variance_mag', registers.DEFAULT_VARIANCE_MAG))
        rospy.loginfo('Parameters set')

    def Convert(self, string):
        try:
            floats = []
            li = list(string.split(","))
            for element in li:
                floats.append(float(element))
            return floats
        except:
            return string
