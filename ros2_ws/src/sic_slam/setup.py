import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sic_slam'


def data_files_tree(dest_prefix, src_root):
    """Mirror src_root's directory tree under share/<package>/dest_prefix."""
    files = []
    for dirpath, _dirnames, filenames in os.walk(src_root):
        if not filenames:
            continue
        rel = os.path.relpath(dirpath, src_root)
        dest = os.path.join('share', package_name, dest_prefix, rel) if rel != '.' \
            else os.path.join('share', package_name, dest_prefix)
        files.append((dest, [os.path.join(dirpath, f) for f in filenames]))
    return files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        *data_files_tree('models', 'models'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Parvu',
    maintainer_email='parvu@example.org',
    description='SIC-SLAM: acoustic-inertial factor-graph SLAM stack for UUV cave exploration (PED2026 prototype build)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sic_slam_perception_bridge.py = sic_slam.sic_slam_perception_bridge:main',
            'sic_slam_graph_backend.py = sic_slam.sic_slam_graph_backend:main',
            'sic_slam_flight_logger.py = sic_slam.sic_slam_flight_logger:main',
            'ping360_sim_node.py = sic_slam.ping360_sim_node:main',
            'training_data_logger.py = sic_slam.training_data_logger:main',
            'sim_info_publisher.py = sic_slam.sim_info_publisher:main',
            'manual_control_node.py = sic_slam.manual_control_node:main',
        ],
    },
)
