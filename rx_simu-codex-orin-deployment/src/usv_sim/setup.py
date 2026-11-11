from setuptools import setup

setup(name='usv_sim', version='0.1.0', packages=['usv_sim'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/usv_sim']),
                  ('share/usv_sim', ['package.xml'])],
      install_requires=['setuptools'],
      entry_points={'console_scripts': ['cloud_filter = usv_sim.cloud_filter:main',
                                        'livox_filter = usv_sim.livox_filter:main',
                                        'gnss_injector = usv_sim.gnss:main',
                                        'map_origin = usv_sim.map_origin:main']})
