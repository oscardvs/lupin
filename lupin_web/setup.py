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
