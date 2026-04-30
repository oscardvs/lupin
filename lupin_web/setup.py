import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'lupin_web'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test', 'web']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files — reachable via `ros2 launch lupin_web lupin_web.launch.py`.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # systemd unit + install script — installed alongside but not auto-activated.
        # The robot operator runs `sudo lupin_web/scripts/install-systemd.sh` manually.
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*.service')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Lupin',
    maintainer_email='o.a.e.devos@student.tudelft.nl',
    description='Browser-based HMI for the MIRTE Master. Static Vite + React + '
                'shadcn frontend talking to the running rosbridge_websocket. '
                'Coexists with the course web interface.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
