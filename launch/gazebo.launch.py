from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_param_builder import load_xacro
from launch_ros.actions import Node
from os.path import join
from pathlib import Path

def generate_launch_description():

    path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    
    gazebo_sim = IncludeLaunchDescription(path, launch_arguments=[("gz_args",  "empty.sdf")])

    robot = ExecuteProcess(
         cmd=["ros2", "run", "ros_gz_sim", "create", "-topic", "robot_description", "-x", "0.0", "-y","0.0", "-z", "1.23"],
        name="spawn robot",
        output="both"
    )

    robot_file = join(get_package_share_directory("maciv2"), "robot_description","maciv2.urdf.xacro")
    robot_xml = load_xacro(Path(robot_file))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description':robot_xml, 
                     'use_sim_time':True}],
    )

    start_controllers  = Node(
                package="controller_manager",
                executable="spawner",
                arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller'],
                output="screen",
            )
    
    move_group = IncludeLaunchDescription(join(get_package_share_directory("maciv2_moveit"), "launch", "move_group.launch.py"))
    rviz = IncludeLaunchDescription(join(get_package_share_directory("maciv2_moveit"), "launch", "moveit_rviz.launch.py"))

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
                   ],
        output='screen'        )
    mg_sim_time = ExecuteProcess(cmd=["ros2", "param", "set", "/move_group", "use_sim_time","True"])
    rviz_sim_time = ExecuteProcess(cmd=["ros2", "param", "set", "/rviz", "use_sim_time","True"])

    return LaunchDescription([gazebo_sim, robot, robot_state_publisher, start_controllers, 
                              move_group, rviz, bridge, mg_sim_time, rviz_sim_time])