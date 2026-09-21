from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont

from .node import AP1ConsoleNode

class DiagnosticsDisplay(QWidget):
    """
    All the diagnostics at the top of the display.
    """

    def __init__(self, node: AP1ConsoleNode, parent=None):
        super().__init__(parent)
        self.ros_node = node

        self.layout : QVBoxLayout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)

        self.content_label = QLabel()
        self.content_label.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)

        font = QFont("Courier New", 10)
        self.content_label.setFont(font)

        self.layout.addWidget(self.content_label)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(100)

    def compose(self):
        with VerticalScroll(id="diagnostics_scroll"):
            yield Static("", id="diagnostics_content")

    def on_mount(self):
        self.content = self.query_one("#diagnostics_content", Static)
        self.set_interval(0.1, self.update_display)

    def update_display(self):
        speed = self.ros_node.current_speed
        turn_angle = self.ros_node.current_turn_angle
        target_speed = self.ros_node.target_speed
        target_loc = self.ros_node.target_location
        motor_power = self.ros_node.motor_power * 100
        brake = self.ros_node.brake * 100
        current_state = self.ros_node.planning_state

        display_text = f"""Actuation:
- Speed:\t{speed:6.2f} m/s
- Turn Angle:\t{turn_angle:6.2f} rads

Console (Output):
- Target Speed:\t{target_speed:6.2f} m/s
- Target Loc:\t{target_loc[0]:6.2f}, {target_loc[1]:6.2f}

Control:
- Motor power:\t{motor_power:6.2f} %
- Brake: \t{brake:6.2f} %

Planning:
- Current state: \t{current_state}
"""
        self.content_label.setText(display_text)
