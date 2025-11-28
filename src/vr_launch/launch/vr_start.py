import os
import sys
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 윈도우에서 Python 스크립트 실행을 위한 헬퍼 함수
    # ament_cmake로 설치된 .py 파일은 실행 권한이 없으므로 python 실행파일을 직접 호출해야 함
    def get_py_path(package_name, script_name):
        return os.path.join(get_package_prefix(package_name), 'lib', package_name, script_name)

    return LaunchDescription([
        # 1. C++ 노드 (vr_node)
        # C++은 컴파일되면 .exe가 되므로 기존 방식대로 실행 가능합니다.
        Node(
            package='vr',
            executable='vr_node',
            name='vr_node',
            output='screen'
        ),

        # 2. Python 노드 (NN.py) - Windows 대응 수정
        Node(
            package=None,  # 패키지 이름을 None으로 설정
            executable=sys.executable,  # 현재 파이썬 실행파일(python.exe) 지정
            arguments=[get_py_path('final', 'NN.py')], # 스크립트 경로를 인자로 전달
            name='NN',
            output='screen'
        ),

        # 3. Python 노드 (visualize.py)
        # Node(
        #     package=None,
        #     executable=sys.executable,
        #     arguments=[get_py_path('final', 'visualize.py')],
        #     name='visualize',
        #     output='screen'
        # ),

        # 4. Python 노드 (point_recorder.py)
        Node(
            package=None,
            executable=sys.executable,
            arguments=['-u', get_py_path('point_recorder', 'point_recorder.py')],
            name='point_recorder',
            output='screen'
        ),

        # 5. Python 노드 (position_check.py)
        # Node(
        #     package=None,
        #     executable=sys.executable,
        #     arguments=[get_py_path('final', 'position_check.py')],
        #     name='position_check',
        #     output='screen'
        # ),
        
        # 6. (만약 있다면) cam.py 노드
        Node(
            package=None,
            executable=sys.executable,
            arguments=[get_py_path('cam_node', 'cam.py')],
            name='cam_node',
            output='screen'
        )
    ])