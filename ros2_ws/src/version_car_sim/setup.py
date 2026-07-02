from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'version_car_sim'


def package_files(directory):
    return [
        path for path in glob(os.path.join(directory, '*'))
        if os.path.isfile(path)
    ]


def package_tree(directory):
    data_files = []
    if not os.path.isdir(directory):
        return data_files

    for root, _, _ in os.walk(directory):
        files = [
            os.path.join(root, name)
            for name in os.listdir(root)
            if os.path.isfile(os.path.join(root, name))
        ]
        if files:
            data_files.append((os.path.join('share', package_name, root), files))
    return data_files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), package_files('launch')),
        (os.path.join('share', package_name, 'models'), package_files('models')),
        (os.path.join('share', package_name, 'worlds'), package_files('worlds')),
        (os.path.join('share', package_name, 'rviz'), package_files('rviz')),
        (os.path.join('share', package_name, 'scripts'), package_files('scripts')),
    ] + package_tree('config'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='version-car',
    maintainer_email='user@example.com',
    description='Gazebo simulation, mapping, planning, and MCU protocol bridge for the version car.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_mapper = version_car_sim.occupancy_mapper:main',
            'astar_planner = version_car_sim.astar_planner:main',
            'local_obstacle_avoidance = version_car_sim.local_obstacle_avoidance:main',
            'd435i_depth_obstacle_mapper = version_car_sim.d435i_depth_obstacle_mapper:main',
            'd435i_astar_planner = version_car_sim.d435i_astar_planner:main',
            'd435i_visual_obstacle_avoidance = version_car_sim.d435i_visual_obstacle_avoidance:main',
            'pointcloud_to_costmap = version_car_sim.pointcloud_to_costmap:main',
            'fast_lio_stub = version_car_sim.fast_lio_stub:main',
            'ego_goal_bridge = version_car_sim.ego_goal_bridge:main',
            'ego_position_cmd_to_twist = version_car_sim.ego_position_cmd_to_twist:main',
            'mcu_protocol_bridge = version_car_sim.mcu_protocol_bridge:main',
            'mapping_drive_node = version_car_sim.mapping_drive_node:main',
            'auto_goal_publisher = version_car_sim.auto_goal_publisher:main',
            'nav2_waypoint_commander = version_car_sim.nav2_waypoint_commander:main',
            'cmd_vel_monitor = version_car_sim.cmd_vel_monitor:main',
            'start_pose_setter = version_car_sim.start_pose_setter:main',
            'trajectory_recorder = version_car_sim.trajectory_recorder:main',
            'gas_field_simulator = version_car_sim.gas_field_simulator:main',
            'spray_simulator = version_car_sim.spray_simulator:main',
            'gas_leak_mobile_manipulator_task = version_car_sim.gas_leak_mobile_manipulator_task:main',
            'gas_leak_car_arm_task = version_car_sim.gas_leak_car_arm_task:main',
            'rebot_arm_minimal_test = version_car_sim.rebot_arm_minimal_test:main',
            'rebot_safe_rl_controller = version_car_sim.rl.rebot_safe_rl_controller:main',
            'rebot_safe_rl_gazebo_trainer = version_car_sim.rl.rebot_safe_rl_gazebo_trainer:main',
        ],
    },
)
