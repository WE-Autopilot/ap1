import sys
import threading
import signal
import subprocess
import time


import rclpy


from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget,
                            QVBoxLayout, QHBoxLayout, QLineEdit,
                            QLabel)
from PyQt6.QtCore import Qt, pyqtSignal

from PyQt6.QtGui import QFont

from .node import AP1ConsoleNode
from .command_output import CommandOutput
from .diagnostics_display import DiagnosticsDisplay
from .visual_path import PathCanvas

def print_help(self):
    self.command_output.add_line('=' * 40)
    self.command_output.add_line('Welcome to AP1 Console')
    self.command_output.add_line('Commands:')
    self.command_output.add_line('\tspeed <value>     - Set target speed (m/s)')
    self.command_output.add_line('\tlocation <x> <y>  - Set target location (m)')
    self.command_output.add_line('\tget speed_profile - Return planned speed profile (m/s)')
    self.command_output.add_line('\tget planned_path  - Return planning output path')
    self.command_output.add_line('\techo topic <topic> [-t seconds] - Echo ROS topic')
    self.command_output.add_line('\treset             - Reset system and simulation (if applicable)')
    self.command_output.add_line('\tclear             - Clear screen')
    self.command_output.add_line('\thelp              - Print this screen')
    self.command_output.add_line('=' * 40)
        
class AP1DebugUI(QMainWindow):
    # me n my homies hate writing css
    # god bless chat:
    # CSS_PATH = 'style.css'

    echo_output_signal = pyqtSignal(str)

    def __init__(self, node: AP1ConsoleNode, app: QApplication):
        super().__init__()
        self.ros_node = node
        self.setWindowTitle("AP1 Console")
        self.resize(1000, 700)
        self.app = app

        # -- Fonts --
        # Main Header Font
        header_font = QFont("Sans Serif", 16)
        header_font.setBold(True)

        # Section Header Font (Bold + Underline)
        section_font = QFont("Sans Serif", 10)
        section_font.setBold(True)
        section_font.setUnderline(True)

        # Central Widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)

        # Header
        header = QLabel("AP1 CONSOLE")
        header.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header.setFont(header_font)
        header.setContentsMargins(0, 0, 0, 5)
        # main_layout.addWidget(header) # removing temp

        # Middle Section
        middle_layout = QHBoxLayout()

        # Left Pane
        left_pane = QWidget()
        left_layout = QVBoxLayout(left_pane)
        left_layout.setContentsMargins(0,0,0,0)

        # Diagnostics
        lbl_diag = QLabel("DIAGNOSTICS")
        lbl_diag.setFont(section_font)
        left_layout.addWidget(lbl_diag)

        self.diagnostics = DiagnosticsDisplay(self.ros_node)
        left_layout.addWidget(self.diagnostics)

        # Path Header
        lbl_path = QLabel("PLANNED PATH")
        lbl_path.setFont(section_font)
        lbl_path.setContentsMargins(0, 10, 0, 0)
        left_layout.addWidget(lbl_path)
        
        self.path_canvas = PathCanvas(self.ros_node)
        left_layout.addWidget(self.path_canvas)

        middle_layout.addWidget(left_pane, stretch=1)

        # Right Pane: Command Output
        right_pane = QWidget()
        right_layout = QVBoxLayout(right_pane)
        right_layout.setContentsMargins(0,0,0,0)

        lbl_cli = QLabel("COMMAND LINE INTERFACE")
        lbl_cli.setStyleSheet("font-weight: bold; text-decoration: underline;")
        # right_layout.addWidget(lbl_cli) # removing

        self.command_output = CommandOutput()
        right_layout.addWidget(self.command_output)

        self.echo_output_signal.connect(self.command_output.add_line)

        self.active_echo_process=None
        self.echo_thread=None

        middle_layout.addWidget(right_pane, stretch=1)
        main_layout.addLayout(middle_layout, stretch=1)

        # Input Footer
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Command...")
        self.command_input.returnPressed.connect(self.on_input_submitted)
        main_layout.addWidget(self.command_input)

        # Help menu on start
        print_help(self)
        self.command_input.setFocus()

    def on_input_submitted(self):
        cmd = self.command_input.text().strip()
        
        if not cmd:
            return

        # Show command
        self.command_output.add_line(f'> {cmd}')
        self.command_input.clear()

        # Execute
        self.execute_command(cmd)

    def on_input_submitted(self):
        cmd = self.command_input.text().strip()
        if not cmd:
            return
        self.command_output.add_line(f'> {cmd}')
        self.command_input.clear()
        self.execute_command(cmd)

    def execute_command(self, cmd: str):
        try:
            parts = cmd.split()
            command = parts[0].lower()

            if command == 'help':
                print_help(self)

            elif command == 'clear':
                self.command_output.history.clear()
                self.command_output.setText('')
                self.command_output.add_line('History cleared') # may remove if annoying

            elif command == 'speed':
                if len(parts) < 2:
                    self.command_output.add_line('Error! Usage: speed <value>')
                else:
                    speed = float(parts[1]) # grab the value
                    self.ros_node.set_target_speed(speed)
                    self.command_output.add_line(f"✓ Target speed set to {speed:.2f} m/s")

            elif command == 'location':
                if len(parts) < 3:
                    self.command_output.add_line('Error! Usage: location <x> <y>')
                else:
                    x, y = float(parts[1]), float(parts[2])
                    self.ros_node.set_target_location(x, y)
                    self.command_output.add_line(f"✓ Target location set to ({x}, {y})")

            elif command == 'get':
                if len(parts) < 2:
                    self.command_output.add_line('Error! Usage: get <speed_profile | planned_path>')
                    return

                subcommand = parts[1].lower()

                if subcommand == 'speed_profile':
                    speed_profile = self.ros_node.speed_profile
                    if not speed_profile:
                        self.command_output.add_line('Speed profile is empty.')
                        return

                    out = ', '.join(f'{spd} m/s' for spd in speed_profile)
                    self.command_output.add_line('{ ' + out + ' }')

                elif subcommand == 'planned_path':
                    path = self.ros_node.target_path
                    if not path:
                        self.command_output.add_line('Planned path is empty.')
                        return

                    out = ', '.join(f'({pt.x:.2f}, {pt.y:.2f})' for pt in path)
                    self.command_output.add_line('{ ' + out + ' }')

                else:
                    self.command_output.add_line('Unknown get command.')

            elif command == 'echo':
                if len(parts) < 3 or parts[1] != 'topic':
                    self.command_output.add_line('Usage: echo topic <topic_name> [-t seconds]')
                    return

                topic_name = parts[2]
                timeout = None

                if '-t' in parts:
                    try:
                        idx = parts.index('-t')
                        timeout = int(parts[idx + 1])
                    except:
                        self.command_output.add_line('Invalid timeout value.')
                        return

                self.command_output.add_line(f'Echoing {topic_name}... (Type "stop" to stop)')

                def run_echo():
                    try:
                        process = subprocess.Popen(
                            ['ros2', 'topic', 'echo', topic_name],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True
                        )

                        self.active_echo_process = process

                        start_time = time.time()

                        for line in process.stdout:

                            if process.poll() is not None:
                                break

                            self.echo_output_signal.emit(line.strip())

                            if timeout and (time.time() - start_time > timeout):
                                process.terminate()
                                self.echo_output_signal.emit('Echo timeout reached.')
                                break

                    except Exception as e:
                        self.echo_output_signal.emit(f'Error: {str(e)}')
                    finally:
                        self.active_echo_process = None
                        self.echo_thread = None

                self.echo_thread = threading.Thread(target=run_echo, daemon=True).start()
                self.echo_thread.start()

            elif command == 'stop':
                if self.active_echo_process is None:
                    self.command_output.add_line('No active echo process to stop')
                else: 
                    try:
                        self.active_echo_process.terminate()
                        self.active_echo_process.wait(timeout=2)
                        self.command_output.add_line('Echo stopped')
                    except:
                        self.active_echo_process.kill()
                        self.command_output.add_line('Echo KILLED')
                    finally:
                        self.active_echo_process = None

            elif command == 'reset':
                self.command_output.add_line("Not yet implemented.")

            else:
                self.command_output.add_line('Command not found.')
        except Exception as e:
            self.command_output.add_line(f"Error: {str(e)}")


    def run(self, node: AP1ConsoleNode):
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        
        if rclpy:
            spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
            spin_thread.start()

        window = self
        window.show()

        sys.exit(self.app.exec())