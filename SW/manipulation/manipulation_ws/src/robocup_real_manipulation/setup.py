from setuptools import setup, find_packages
from glob import glob
setup(name='robocup_real_manipulation',version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/robocup_real_manipulation']),
 ('share/robocup_real_manipulation',['package.xml']),
 ('share/robocup_real_manipulation/config',glob('config/*.yaml')),
 ('share/robocup_real_manipulation/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='INHA Robotics',
 maintainer_email='robotics@example.com',description='Physical rule-based manipulation',license='Apache-2.0',
 entry_points={'console_scripts':[
 'mock_coordinator=robocup_real_manipulation.mock_node:main',
 'coordinator=robocup_real_manipulation.node:main',
 'head_input_adapter=robocup_real_manipulation.head_input_adapter:main',
 'wrist_frontend=robocup_real_manipulation.wrist_frontend:main',
 'phase1_pick=robocup_real_manipulation.phase1_client:main',
 'piper_bridge=robocup_real_manipulation.piper_bridge:main',
 'test_sender=robocup_real_manipulation.test_sender:main',
 'prepare_description=robocup_real_manipulation.description:main',
 'tcp_calibration=robocup_real_manipulation.tcp_calibration:main']})
