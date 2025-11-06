from rclpy.impl.rcutils_logger import RcutilsLogger
from ament_index_python.packages import get_package_share_directory
import os
import yaml


class ConfigLoader:
    def __init__(self, package_name: str, file_path: str) -> None:

        self.logger = RcutilsLogger(self.__class__.__name__)

        # Get the package share directory
        self._package_share_dir = get_package_share_directory(package_name)

        # Construct path to config file
        self._config_file = os.path.join(self._package_share_dir, file_path)

        try:
            with open(self._config_file, 'r') as f:
                self._config = yaml.safe_load(f)
                self.logger.info('Config loaded successfully')
        except Exception as e:
            self.logger.error(f'Failed to load config: {str(e)}')

    def load_config(self):
        return self._config

    def load_docking_config(self):
        # Pose of the dock
        x,y,theta = self._config['/*/docking_server']['ros__parameters']['charging_1']['pose']

        # Stagging offset of the dock
        staging_x_offset = self._config['/*/docking_server']['ros__parameters']['jackal_dock']['staging_x_offset']

        # Actual value of staging pose. (dockig and undock happens in straight line along x)
        staging_x = x + staging_x_offset

        return x, y, theta, staging_x
