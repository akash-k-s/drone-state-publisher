import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='drone_state_publisher',
            executable='drone_missions',
            name='drone_missions_node',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
