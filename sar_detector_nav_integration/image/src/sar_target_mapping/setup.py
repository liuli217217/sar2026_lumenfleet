# DETECTOR_NAV_INTEGRATION: new package, independent of the navigation controller.
from setuptools import setup
setup(name='sar_target_mapping', version='0.1.0', packages=['sar_target_mapping'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/sar_target_mapping']),
                  ('share/sar_target_mapping', ['package.xml']),
                  ('share/sar_target_mapping/launch', ['launch/detector_mapping.launch.py'])],
      install_requires=['setuptools'], license='Apache-2.0',
      entry_points={'console_scripts': ['target_mapper = sar_target_mapping.node:main']})
