from setuptools import setup
from glob import glob
setup(name='robocup_gazebo_manipulation',version='0.1.0',packages=['robocup_gazebo_manipulation'],
 data_files=[('share/ament_index/resource_index/packages',['resource/robocup_gazebo_manipulation']),
 ('share/robocup_gazebo_manipulation',['package.xml']),('share/robocup_gazebo_manipulation/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='INHA United',maintainer_email='robot@example.invalid',
 description='Gazebo Harmonic manipulation adapter with existing navigation model',license='Apache-2.0',
 entry_points={'console_scripts':['controller=robocup_gazebo_manipulation.controller:main',
 'generate=robocup_gazebo_manipulation.generate:main','demo=robocup_gazebo_manipulation.demo:main','probe=robocup_gazebo_manipulation.probe:main','tutorial=robocup_gazebo_manipulation.tutorial:main','task=robocup_gazebo_manipulation.task:main']})
