import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node # ★ 추가: Node 실행을 위한 라이브러리

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_barrier_sim = get_package_share_directory('barrier_sim')

    # 1. 가제보 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # 여기에 이전에 꾸미신 asphalt_plane.sdf나 empty.sdf가 들어가 있을 겁니다.
        launch_arguments={'gz_args': '-r empty.sdf'}.items(), 
    )

    # 2. 로봇(차단기) 모델 스폰
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'parking_barrier',
            '-file', os.path.join(pkg_barrier_sim, 'urdf', 'barrier.urdf'),
            '-z', '0.1'
        ],
        output='screen'
    )

    # ★ 3. 통신 브리지 노드 추가 ★ (이 부분이 핵심입니다)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/barrier_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/car_cmd@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        output='screen'
    )

    # LaunchDescription에 세 가지 모두 리턴
    return LaunchDescription([
        gazebo,
        spawn,
        bridge  # ★ 브리지 노드 실행 추가
    ])