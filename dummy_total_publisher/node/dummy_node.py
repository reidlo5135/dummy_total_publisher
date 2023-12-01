from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.publisher import Publisher
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

from robot_status_msgs.msg import VelocityStatus


RCLPY_NODE_NAME: str = 'dummy_navigate_to_pose'

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
        
        
    def __main_timer_cb(self) -> None:
        self.__publish_battery_state()
        pass
    
    
    def __publish_battery_state(self) -> None:
        battery_state: BatteryState = BatteryState()
        
    
    
__all__ = ['dummy_node']