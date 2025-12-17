# display.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. 获取本包 share 目录（config 与 launch 都在这里安装）
    my_arm_config_path = get_package_share_directory('my_arm_config')

    # 模型文件路径（MoveIt 生成的 xacro 位于 config/）
    xacro_file = os.path.join(my_arm_config_path, 'config', 'my_arm_with_gripper.urdf.xacro')

    # 2. 解析 Xacro 得到机器人描述
    robot_desc = xacro.process_file(xacro_file).toxml()

    # 3. 配置 RViz 文件路径（仅包含 RobotModel 显示）
    rviz_config_file = os.path.join(my_arm_config_path, 'config', 'display.rviz')

    # 4. 启动 Robot State Publisher (RSP)  
    # RSP 将 URDF 内容发布到 /robot_description 参数
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 5. 启动 Joint State Publisher (JSP)
    # JSP 模拟关节值变化（如果没有真正的硬件或仿真环境）
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui', # 或 joint_state_publisher
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    
    # 6. 启动 RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'robot_description': robot_desc}],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])