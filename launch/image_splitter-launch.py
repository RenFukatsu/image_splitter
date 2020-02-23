import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='image_splitter',
            node_executable='image_splitter',
            output='screen',
            node_name='image_splitter',
            remappings=[('image', 'usb_cam/image_raw')],
            parameters=['parameters.yaml']
            # parameters=[{'hz':0.0}, {'save_dir_name':'/home/amsl/colcon_ws/src/human_tracking_using_robots/bagfiles/fox/images/alone1'}]
        )
    ])