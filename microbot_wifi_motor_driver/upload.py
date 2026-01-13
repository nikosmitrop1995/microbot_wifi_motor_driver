"""Upload module for microbot_wifi_motor_driver package."""
import subprocess
from typing import List, Optional

from ament_index_python.packages import get_package_share_path

import rclpy
from rclpy.node import Node


class Upload(Node):
    """Node for uploading the firmware."""

    def __init__(self) -> None:
        """Initialize the upload node."""
        super().__init__('upload_node')
        self.run_cmd()

    def run_cmd(self) -> None:
        """Run the platformio upload command."""
        package_dir = get_package_share_path('microbot_wifi_motor_driver')
        cmd = f'cd {package_dir} && pio run -t upload'
        subprocess.run(cmd, shell=True, check=False)


def main(args: Optional[List[str]] = None) -> None:
    """Run the upload script."""
    rclpy.init(args=args)
    Upload()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
