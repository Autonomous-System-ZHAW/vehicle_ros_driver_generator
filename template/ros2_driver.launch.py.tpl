from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_frame_id_arg = DeclareLaunchArgument(
        'base_frame_id', default_value='base_link',
        description='Frame ID used in published message headers')

    loop_rate_arg = DeclareLaunchArgument(
        'loop_rate', default_value='50.0',
        description='Node timer loop rate (Hz)')

    report_timeout_ms_arg = DeclareLaunchArgument(
        'report_timeout_ms', default_value='1000',
        description='Timeout for report parsing in milliseconds')

    command_timeout_ms_arg = DeclareLaunchArgument(
        'command_timeout_ms', default_value='1000',
        description='Timeout for control command publishing in milliseconds')

    # Report parser node (subscribes to input/can_rx and publishes parsed topics)
    report_parser_node = Node(
        package='%(package_prefix)s_%(car_type)s_driver',
        executable='%(package_prefix)s_%(car_type)s_driver_report_parser_node',
        name='report_parser',
        output='screen',
        parameters=[{
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'loop_rate': LaunchConfiguration('loop_rate'),
            'report_timeout_ms': LaunchConfiguration('report_timeout_ms'),
        }],
        # Example remappings (uncomment/customize as needed):
        # remappings=[('input/can_rx', '/from_can_bus'), ('input/is_publish', '/to_can_bus')]
    )

    # Control command node (publishes CAN frames to output/can_tx)
    control_command_node = Node(
        package='%(package_prefix)s_%(car_type)s_driver',
        executable='%(package_prefix)s_%(car_type)s_driver_control_command_node',
        name='control_command',
        output='screen',
        parameters=[{
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'loop_rate': LaunchConfiguration('loop_rate'),
            'command_timeout_ms': LaunchConfiguration('command_timeout_ms'),
        }],
        # Example remappings (uncomment/customize as needed):
        # remappings=[('output/can_tx', '/your_can_tx_topic'), ('input/engage', '/your_engage_topic')]
    )

    return LaunchDescription([
        base_frame_id_arg,
        loop_rate_arg,
        report_timeout_ms_arg,
        command_timeout_ms_arg,
        report_parser_node,
        control_command_node,
    ])
