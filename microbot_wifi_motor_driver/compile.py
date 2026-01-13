"""Compile module for microbot_wifi_motor_driver package."""
import subprocess
from typing import List, Optional

from ament_index_python.packages import get_package_share_path

import rclpy
from rclpy.node import Node


class Compile(Node):
    """Node for compiling the firmware."""

    def __init__(self) -> None:
        """Initialize the compile node."""
        super().__init__('compile')
        self.run_cmd()

    def run_cmd(self) -> None:
        """Run the platformio compile command."""
        package_dir = get_package_share_path('microbot_wifi_motor_driver')
        cmd = f'cd {package_dir} && pio run'
        subprocess.run(cmd, shell=True, check=False)


def main(args: Optional[List[str]] = None) -> None:
    """Run the compile script."""
    rclpy.init(args=args)
    Compile()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
