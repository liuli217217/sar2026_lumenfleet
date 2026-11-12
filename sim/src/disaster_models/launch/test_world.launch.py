import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_my_disaster_models = get_package_share_directory('disaster_models')

    # ---> UPDATE THIS LINE <---
    world_path = os.path.join(
        pkg_my_disaster_models,
        'worlds',
        'test_disaster_world.sdf'  # Load your new office world
    )
    models_path = os.path.join(pkg_my_disaster_models, 'models')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_path + ' -r'}.items() 
    )

    return LaunchDescription([
        gz_resource_path,
        gz_sim_launch
    ])