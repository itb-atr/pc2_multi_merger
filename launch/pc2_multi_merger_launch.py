from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    scanner_merger_node = Node(package="pc2_multi_merger", executable='pc2_multi_merger', name='pc2_multi_merger',
                               parameters=[{'destination_frame': 'base_link',
                                            'cloud_destination_topic': '/merged_pointcloud',
                                            'pc2_topics': '/front_scan /rear_scan'}],
                               output='screen')
    ld.add_action(scanner_merger_node)
    return ld
