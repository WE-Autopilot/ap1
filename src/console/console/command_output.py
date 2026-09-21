from collections import deque

from PyQt6.QtWidgets import QTextEdit
from PyQt6.QtGui import QFont

class CommandOutput(QTextEdit):
    """Display for cmd history + outputs"""

    def __init__(self, parent=None):
        super().__init__(parent)

        # Properties
        self.setReadOnly(True)
        font = QFont("Courier New", 10)
        self.setFont(font)

        self.history = deque(maxlen=100)

    def add_line(self, text: str):
        self.history.append(text)
        self.setText('\n'.join(self.history))

        # Scroll bar
        self.verticalScrollBar().setValue(self.verticalScrollBar().maximum())