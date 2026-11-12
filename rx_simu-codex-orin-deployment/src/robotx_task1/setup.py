from setuptools import setup

setup(name='robotx_task1', version='0.1.0', packages=['robotx_task1'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/robotx_task1']),
                  ('share/robotx_task1', ['package.xml']),
                  ('share/robotx_task1/launch', ['launch/task1_usv.launch.py']),
                  ('share/robotx_task1/config', ['config/task1_usv_real.yaml'])],
      entry_points={'console_scripts': [
          'known_observer = robotx_task1.observer:main',
          'mission = robotx_task1.mission:main',
          'traffic_guard = robotx_task1.traffic_guard:main',
          'judge = robotx_task1.judge:main',
          'survey_check = robotx_task1.survey_check:main',
          'survey_analyze = robotx_task1.survey_bag:main',
          'buoy_mapper = robotx_task1.buoy_mapper:main',
          'usv_mission = robotx_task1.usv_mission:main',
          'robocommand = robotx_task1.robocommand:main',
          'fsm_bridge = robotx_task1.fsm_bridge:main']})
