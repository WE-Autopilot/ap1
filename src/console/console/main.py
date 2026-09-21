import sys

import rclpy

from PyQt6.QtWidgets import QApplication

from .control_ui import AP1DebugUI
from .node import AP1ConsoleNode

# main time bbgl
def main(args=None):
    rclpy.init(args=args)
    ros_node = AP1ConsoleNode()
    app = AP1DebugUI(ros_node, QApplication(sys.argv))

    try:
        # run
        app.run(ros_node)
    finally:
        # teardown
        ros_node.destroy_node()
        rclpy.shutdown()

# bismAllah
if __name__ == '__main__':
    main()
