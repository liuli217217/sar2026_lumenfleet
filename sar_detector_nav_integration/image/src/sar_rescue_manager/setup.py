# DETECTOR_NAV_INTEGRATION: mission orchestration package.
from glob import glob
from setuptools import setup
setup(name='sar_rescue_manager', version='0.1.0', packages=['sar_rescue_manager'],
 data_files=[('share/ament_index/resource_index/packages', ['resource/sar_rescue_manager']),
 ('share/sar_rescue_manager', ['package.xml']), ('share/sar_rescue_manager/launch', glob('launch/*.launch.py'))],
 install_requires=['setuptools'], license='Apache-2.0',
 entry_points={'console_scripts': [
 'rescue_manager = sar_rescue_manager.node:main',
 'managed_explorer = sar_rescue_manager.managed_explorer:main',
 'mock_rescue = sar_rescue_manager.mock_rescue:main']})
