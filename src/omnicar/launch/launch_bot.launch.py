import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command


def generate_launch_description():
    package_name='omnicar'

    controller_params = os.path.join(get_package_share_directory(package_name), 'config','my_controllers.yaml')
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    rsp = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py')]), 
      launch_arguments={'use_sim_time': 'false', 'sim_mode': 'false'}.items()
    )

    joystick = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','joystick.launch.py')]), 
    )

    # ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py port:=/dev/clearpath/gps baud:=19200
    gps = Node(
      package='nmea_navsat_driver',
      executable='nmea_serial_driver',
      parameters=[{'port': '/dev/ttyACM1', 'baud': 9600}],
      output="both",
    )

    imu = Node(
      package='ros2_mpu6050',
      executable='ros2_mpu6050',
    )

    imu_visualiser = Node(
      package='imu_filter_madgwick',
      executable='imu_filter_madgwick_node',
      parameters=[{'use_mag': False, 'publish_tf': True, 'world_frame': 'enu'}],
      remappings=[("/imu/data_raw", "/imu/mpu6050")],
    )

    camera = Node(
      package='usb_cam',
      executable='usb_cam_node_exe',
      name='usb_cam_node',
      output='screen',
      parameters=[{
        'video_device': '/dev/video0',
        'image_width': 1920,
        'image_height': 1080,
        "frame_rate": 30,
        'io_method': 'mmap',
        "retry_on_error": True,
        "pixel_format": "mjpeg2rgb"
      }]
    ), 


    controller_manager = Node(
      package='controller_manager',
      executable='ros2_control_node',
      parameters=[controller_params],
      output="both",
      remappings=[
          ("~/robot_description", "/robot_description"),
          # ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),
      ],    
    )
    diff_drive_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["diff_cont"],
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
      event_handler=OnProcessStart(
        target_action=controller_manager,
        on_start=[joint_broad_spawner],
      )
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
      event_handler=OnProcessStart(
        target_action=controller_manager,
        on_start=[diff_drive_spawner],
      )
    )

    delayed_controller_manager = TimerAction(period=5.0,actions=[controller_manager])
   
   
    return LaunchDescription([
      rsp,
      gps,
      imu,
      imu_visualiser,
      camera,
      delayed_controller_manager,
      delayed_diff_drive_spawner,
      delayed_joint_broad_spawner,
      joystick
    ])
