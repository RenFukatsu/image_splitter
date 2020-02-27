import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='image_splitter',
            node_executable='image_splitter',
            output='screen',
            node_name='image_splitter',
            remappings=[('image', 'your_image_topic')],
            parameters=[{'hz':0.0, 'save_dir_path':'your_save_dir'}]
        ),
    ])