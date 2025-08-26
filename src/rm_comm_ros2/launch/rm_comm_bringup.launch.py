from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	port = LaunchConfiguration('port', default='/dev/ttyACM0')
	baud = LaunchConfiguration('baud', default='115200')
	reopen_interval_ms = LaunchConfiguration('reopen_interval_ms', default='500')
	tx_hz = LaunchConfiguration('tx_hz', default='100.0')
	use_shm = LaunchConfiguration('use_shm', default='false')
	shm_name = LaunchConfiguration('shm_name', default='/rm_comm_shm')

	return LaunchDescription([
		DeclareLaunchArgument('port', default_value=port),
		DeclareLaunchArgument('baud', default_value=baud),
		DeclareLaunchArgument('reopen_interval_ms', default_value=reopen_interval_ms),
		DeclareLaunchArgument('tx_hz', default_value=tx_hz),
		DeclareLaunchArgument('use_shm', default_value=use_shm),
		DeclareLaunchArgument('shm_name', default_value=shm_name),

		# 直接使用本包串口节点，无需 serial_driver
		Node(
			package='rm_comm_ros2',
			executable='serial_rw_node',
			name='serial_rw_node',
			parameters=[{
				'port': port,
				'baud': baud,
				'reopen_interval_ms': reopen_interval_ms,
				'use_shm': use_shm,
				'shm_name': shm_name,
			}],
		),

		Node(
			package='rm_comm_ros2',
			executable='handler_node',
			name='handler_node',
			parameters=[{'tx_hz': tx_hz, 'use_shm': use_shm, 'shm_name': shm_name}],
		),
	]) 