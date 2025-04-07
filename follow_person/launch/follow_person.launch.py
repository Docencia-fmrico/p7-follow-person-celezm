from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # pkg_dir = get_package_share_directory('camera')
    # param_file = os.path.join(pkg_dir, 'config', 'params.yaml')  # ← F841: no se usa

    detector_cmd = Node(
        package='follow_person',
        executable='yolo_to_tf',
        output='screen',
        remappings=[
            ('/input_detections', '/detections_3d')
        ]
    )

    follow_person_cmd = Node(
        package='follow_person',
        executable='person_seeker',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(detector_cmd)
    ld.add_action(follow_person_cmd)

    return ld
