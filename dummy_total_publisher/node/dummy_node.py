import numpy

from datetime import datetime

from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.publisher import Publisher
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from robot_status_msgs.msg import VelocityStatus


RCLPY_NODE_NAME: str = 'dummy_total_publisher'
DEFAULT_FLOAT: float = 0.0
DEFAULT_INT: int = 0


class DummyTotalPublisher(Node):
    
    def __init__(self) -> None:
        super().__init__(RCLPY_NODE_NAME)
        self.get_logger().info(f'===== [{RCLPY_NODE_NAME}] created =====')
        
        __main_timer_period_sec: float = 1.0
        self.__main_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__main_timer: Timer = self.create_timer(timer_period_sec = __main_timer_period_sec, callback = self.__main_timer_cb, callback_group = self.__main_timer_cb_group)
        
        self.__battery_state_publisher_topic: str = '/battery/state'
        self.__battery_state_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__battery_state_publisher: Publisher = self.create_publisher(
            msg_type = BatteryState,
            topic = self.__battery_state_publisher_topic,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__battery_state_publisher_cb_group
        )
        
        self.__velocity_state_publisher_topic: str = '/velocity/state'
        self.__velocity_state_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__velocity_state_publisher: Publisher = self.create_publisher(
            msg_type = VelocityStatus,
            topic = self.__velocity_state_publisher_topic,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__velocity_state_publisher_cb_group
        )
        
        self.__scan_multi_publisher_topic: str = '/scan/multi'
        self.__scan_multi_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__scan_multi_publisher: Publisher = self.create_publisher(
            msg_type = LaserScan,
            topic = self.__scan_multi_publisher_topic,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__scan_multi_publisher_cb_group
        )
        
        self.__imu_data_publisher_topic: str = '/imu/data'
        self.__imu_data_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__imu_data_publisher: Publisher = self.create_publisher(
            msg_type = Imu,
            topic = self.__imu_data_publisher_topic,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__imu_data_publisher_cb_group
        )
        
        self.__ublox_fix_publisher_topic: str = '/ublox/fix'
        self.__ublox_fix_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__ublox_fix_publisher: Publisher = self.create_publisher(
            msg_type = NavSatFix,
            topic = self.__ublox_fix_publisher_topic,
            qos_profile = qos_profile_sensor_data,
            callback_group = self.__ublox_fix_publisher_cb_group
        )

        self.__rtt_odom_publisher_topic: str= '/rtt_odom'
        self.__rtt_odom_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.__rtt_odom_publisher: Publisher = self.create_publisher(
            msg_type = PoseStamped,
            topic = self.__rtt_odom_publisher_topic,
            qos_profile = qos_profile_system_default,
            callback_group = self.__rtt_odom_publisher_cb_group
        )
    
    def __get_current_datetime(self) -> str:
        __current_datetime: datetime = datetime.now()
        __formatted_datetime: str = __current_datetime.strftime("%Y%m%d%H%M%S")

        return __formatted_datetime
        
    def __main_timer_cb(self) -> None:
        self.__publish_battery_state()
        self.__publish_velocity_state()
        self.__publish_scan_multi()
        self.__publish_imu_data()
        self.__publish_ublox_fix()
        self.__publish_rtt_odom()
        
    
    def __build_time(self) -> Time:
        stamp: Time = Time()
        stamp.sec = DEFAULT_INT
        stamp.nanosec = DEFAULT_INT

        return stamp


    def __build_header(self, frame_id: str) -> Header:
        header: Header = Header()
        header.stamp = self.__build_time()
        header.frame_id = frame_id

        return header
        
    
    def __publish_battery_state(self) -> None:
        battery_state: BatteryState = BatteryState()
        battery_state.header = self.__build_header('battery_state')
        battery_state.voltage = DEFAULT_FLOAT
        battery_state.current = DEFAULT_FLOAT
        battery_state.charge = DEFAULT_FLOAT
        battery_state.capacity = DEFAULT_FLOAT
        battery_state.design_capacity = DEFAULT_FLOAT
        battery_state.percentage = 59.0
        battery_state.power_supply_status = DEFAULT_INT
        battery_state.power_supply_health = DEFAULT_INT
        battery_state.power_supply_technology = DEFAULT_INT
        
        self.__battery_state_publisher.publish(battery_state)


    def __publish_velocity_state(self) -> None:
        velocity_status: VelocityStatus = VelocityStatus()
        velocity_status.current_velocity = 5.9
        velocity_status.distance = float(self.__get_current_datetime())

        self.__velocity_state_publisher.publish(velocity_status)
    

    def __publish_scan_multi(self) -> None:
        laser_scan: LaserScan = LaserScan()
        laser_scan.header = self.__build_header('scan')
        laser_scan.angle_min = DEFAULT_FLOAT
        laser_scan.angle_max = DEFAULT_FLOAT
        laser_scan.angle_increment = DEFAULT_FLOAT
        laser_scan.time_increment = DEFAULT_FLOAT
        laser_scan.scan_time = DEFAULT_FLOAT
        laser_scan.range_min = DEFAULT_FLOAT
        laser_scan.range_max = DEFAULT_FLOAT

        ranges: list = [0.0, 1.1, 2.2]
        laser_scan.ranges = ranges

        intensities: list = [5.5, 4.4, 3.3]
        laser_scan.intensities = intensities

        self.__scan_multi_publisher.publish(laser_scan)

    
    def __build_quaternion(self) -> Quaternion:
        quaternion: Quaternion = Quaternion()
        quaternion.x = DEFAULT_FLOAT
        quaternion.y = 4.5
        quaternion.z = DEFAULT_FLOAT
        quaternion.w = DEFAULT_FLOAT

        return quaternion

    def __build_vector3(self) -> Vector3:
        vector3: Vector3 = Vector3()
        vector3.x = DEFAULT_FLOAT
        vector3.y = DEFAULT_FLOAT
        vector3.z = DEFAULT_FLOAT

        return vector3
    

    def __publish_imu_data(self) -> None:
        imu: Imu = Imu()
        imu.header = self.__build_header('imu')
        imu.orientation = self.__build_quaternion()
        imu.orientation_covariance = numpy.zeros(9, dtype = numpy.float)
        imu.angular_velocity = self.__build_vector3()
        imu.angular_velocity_covariance = numpy.zeros(9, dtype = numpy.float)
        imu.linear_acceleration = self.__build_vector3()
        imu.linear_acceleration_covariance = numpy.zeros(9, dtype = numpy.float)

        self.__imu_data_publisher.publish(imu)

    
    def __publish_ublox_fix(self) -> None:
        nav_sat_fix: NavSatFix = NavSatFix()
        nav_sat_fix.longitude = 128.85798356757832
        nav_sat_fix.latitude = 35.1576297715377
        
        self.__ublox_fix_publisher.publish(nav_sat_fix)


    def __build_point(self) -> Point:
        point: Point = Point()
        point.x = DEFAULT_FLOAT
        point.y = DEFAULT_FLOAT
        point.z = DEFAULT_FLOAT

        return point
    

    def __build_pose(self) -> Pose:
        pose: Pose = Pose()
        pose.position = self.__build_point()
        pose.orientation = self.__build_quaternion()

        return pose
    

    def __publish_rtt_odom(self) -> None:
        pose_stamped: PoseStamped = PoseStamped()
        pose_stamped.header = self.__build_header('rtt_odom')
        pose_stamped.pose = self.__build_pose()

        self.__rtt_odom_publisher.publish(pose_stamped)

    
__all__ = ['dummy_node']