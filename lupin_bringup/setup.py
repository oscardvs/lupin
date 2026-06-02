import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'lupin_bringup'

data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py') + glob('launch/*.launch.xml')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml') + glob('config/*.json') + glob('config/*.xml.in')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/systemd', glob('systemd/*.service')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
        ('share/' + package_name + '/udev', glob('udev/*.rules')),
    ]

# Recursively install everything under models/ (AprilTag textures + material
# scripts) preserving the directory structure, so Gazebo can resolve
# `model://apriltags/...` URIs once share/lupin_bringup/models is on
# GAZEBO_MODEL_PATH.
for root, dirs, files in os.walk('models'):
    for file in files:
        source_file = os.path.join(root, file)
        # Replicate the directory structure inside install/share/lupin_bringup/
        dest_dir = os.path.join('share', package_name, root)
        data_files.append((dest_dir, [source_file]))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Lupin',
    maintainer_email='o.a.e.devos@student.tudelft.nl',
    description='Top-level launch files, params, and system glue for MDP Team Lupin.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Sim helper: publishes a synthetic /amcl_pose so the v2 mission
            # orchestrator's PREPARE.LOCALIZING gate clears in slam_toolbox
            # mode (no real AMCL in the chain). Hardware doesn't need it.
            'seed_amcl_pose = lupin_bringup.seed_amcl_pose:main',
            # Launch sentinel — blocks until tf 'base_link' is resolvable
            # against 'odom'. Used to gate Nav2 lifecycle start on hardware,
            # where a cold DDS-over-WiFi /tf subscription needs ~5–10 s to
            # warm up before Nav2's costmap activation can succeed.
            'wait_for_tf = lupin_bringup.wait_for_tf:main',
            # Sim helper: publishes a draining BatteryState on
            # /io/power/power_watcher (the topic the real MIRTE power
            # watcher uses) so Nav2's IsBatteryLow and the mission
            # orchestrator's BatteryMonitor have data in sim.
            'sim_battery_publisher = lupin_bringup.sim_battery_publisher:main',
        ],
    },
)
