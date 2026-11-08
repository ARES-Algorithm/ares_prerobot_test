from glob import glob
from setuptools import setup

setup(
    name='ares_prerobot_test', version='0.2.0', packages=['ares_prerobot_test'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ares_prerobot_test']),
        ('share/ares_prerobot_test', ['package.xml']),
        ('share/ares_prerobot_test/config', glob('config/*.yaml')),
        ('share/ares_prerobot_test/launch', glob('launch/*.py')),
        ('share/ares_prerobot_test/web', glob('web/*')),
    ],
    entry_points={'console_scripts': [
        'chassis_controller = ares_prerobot_test.controller:main',
        'chassis_cli = ares_prerobot_test.cli:main',
    ]},
)
