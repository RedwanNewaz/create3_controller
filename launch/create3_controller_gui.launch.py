from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch_ros.actions import Node

def generate_launch_description():

    create3_controller_gui = Node(
        package='create3_controller',
        executable='create3_controller_gui',
        name='create3_controller_gui',
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
    )


    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
    )
    ld = LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    ld.add_action(create3_controller_gui)

    return ld
