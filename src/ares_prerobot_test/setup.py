from setuptools import setup
from glob import glob
setup(name='ares_prerobot_test',version='0.2.0',packages=['ares_prerobot_test'],
 data_files=[('share/ament_index/resource_index/packages',['resource/ares_prerobot_test']),('share/ares_prerobot_test',['package.xml']),('share/ares_prerobot_test/config',glob('config/*')),('share/ares_prerobot_test/launch',glob('launch/*')),('share/ares_prerobot_test/web',glob('web/*'))],
 entry_points={'console_scripts':['test_runner = ares_prerobot_test.runner:main','analyze = ares_prerobot_test.analysis:main','mock_chassis = ares_prerobot_test.mock:main','chassis_controller = ares_prerobot_test.controller:main','chassis_cli = ares_prerobot_test.cli:main']})
