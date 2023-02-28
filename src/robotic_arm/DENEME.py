from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindPackageShare, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set the path to the URDF file
    robot_description_path = LaunchConfiguration('robot_description_path', default=get_package_share_directory('my_robot_description') + '/urdf/my_robot.urdf')

    # Set the path to the camera URDF file
    camera_description_path = LaunchConfiguration('camera_description_path', default=get_package_share_directory('my_robot_description') + '/urdf/my_camera.urdf.xacro')

    # Set the path to the Gazebo world file
    world_file_path = LaunchConfiguration('world_file_path', default=get_package_share_directory('my_robot_description') + '/worlds/my_world.world')

    # Load the robot description
    robot_description = {'robot_description': Command(['xacro ', robot_description_path])}

    # Launch the Gazebo simulator
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py'
        ]),
        launch_arguments={'world': world_file_path}.items(),
    )

    # Spawn the robot model in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-file', robot_description_path],
    )

    # Add the camera to the robot model
    add_camera = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', 'my_camera_description', '-entity', 'my_camera', '-x', '0.5', '-z', '1.0'],
        env={'GAZEBO_MODEL_PATH': get_package_share_directory('my_robot_description') + '/models'},
    )
    add_camera.set_parameter_file(camera_description_path)

    # Start the camera plugin
    camera_plugin = Node(
        package='gazebo_ros', executable='camera',
        arguments=['-camera_name', 'my_camera', '-topic', 'my_camera/image_raw'],
        remappings=[('camera_info', 'my_camera/camera_info')],
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_entity,
        add_camera,
        camera_plugin,
    ])
