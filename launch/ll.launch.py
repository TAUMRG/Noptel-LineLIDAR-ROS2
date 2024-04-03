from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():

   return LaunchDescription([
      Node(
         package='Noptel_LineLIDAR_ROS2',
         executable='ll_ros2',
         name='linelidar',
         parameters=[
         	{'addr': '192.168.10.98'},
         	{'freq': 20},
         	{'encoder_step_count': 1199},
         	{'threshold': 3000},
         ]
      ),
   ])
