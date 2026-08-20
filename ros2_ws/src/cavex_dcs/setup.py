from setuptools import find_packages, setup

package_name = 'cavex_dcs'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CaveX Explorer Pro',
    maintainer_email='petrisor.parvu@upb.ro',
    description='Drift/Current Suppression controller for the BlueROV2',
    license='MIT',
    tests_require=['pytest'],
    # extras_require is what modern setuptools (which dropped the 'test'
    # command and tests_require) actually preserves in the Distribution
    # object; colcon's ament_python pytest-detection reads this to select
    # the pytest test step over falling back to `python -m unittest`.
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'dcs_controller = cavex_dcs.dcs_controller:main',
        ],
    },
)
