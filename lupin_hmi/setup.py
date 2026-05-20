import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lupin_hmi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Add this line to copy your YAML file!
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # systemd unit + install script for the robot-side gripper bridge.
        # Operator runs `sudo lupin_hmi/scripts/install-systemd.sh` from the
        # source tree on the robot — mirrors the lupin_web pattern.
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*.service')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koen_vogels',
    maintainer_email='koen_vogels@todo.todo',
    description='Remote control and takeover logic for MIRTE Master',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_mux = lupin_hmi.cmd_vel_mux:main',
            'arm_teleop = lupin_hmi.arm_teleop:main',
            'arm_sim_shim = lupin_hmi.arm_sim_shim:main',
            'arm_preset_server = lupin_hmi.arm_preset_server:main',
            'arm_calibrate_server = lupin_hmi.arm_calibrate_server:main',
            'gripper_action_bridge = lupin_hmi.gripper_action_bridge:main',
            # One-shot called by lupin-auto-home.service at boot. Waits for
            # the arm controllers + preset service to come up, then moves
            # the arm to the 'home' preset (low-gravity-load rest pose).
            'auto_home = lupin_hmi.auto_home:main',
        ],
    },
)