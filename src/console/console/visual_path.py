"""
Visualizes the car's path and surrounding features on a 2D canvas.

Coordinate conventions:
  World space:  +X = FORWARD, +Y = LEFT  (meters)
  Screen space: +X = RIGHT,   +Y = DOWN  (pixels)

Transform pipeline:
  world_point
    → world_to_canvas()   scale meters → logical canvas units, flip axes
    → canvas_to_screen()  scale logical units → screen pixels
"""

from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QPen, QBrush
from PyQt6.QtCore import Qt, QTimer

from .node import AP1ConsoleNode

# || DEFAULTS
DEFAULT_FEATURE_TYPE = (
    "stop_sign"  # TEMPORARY, entities don't contain types yet so this is for now
)

# || Colors
WHITE = Qt.GlobalColor.white
DIM = Qt.GlobalColor.darkGray
RED = Qt.GlobalColor.red
GREEN = Qt.GlobalColor.green
YELLOW = Qt.GlobalColor.yellow
PURPLE = Qt.GlobalColor.darkMagenta
BLUE = Qt.GlobalColor.blue
BLACK = Qt.GlobalColor.black

FEATURE_COLORS = {
    "stop_sign": RED,
    "traffic_light": GREEN,
    "stop_line": YELLOW,
    "yield_sign": BLUE,
}

# || Canvas constants
# The logical canvas is a fixed 60×60 unit grid. The car sits at the
# bottom-center: (ORIGIN_X, ORIGIN_Y) in logical units.
CANVAS_W = 60  # logical width  (units)
CANVAS_H = 60  # logical height (units)
ORIGIN_X = CANVAS_W // 2  # horizontal center — car is laterally centered
ORIGIN_Y = CANVAS_H - 1  # near bottom edge  — car is at the bottom

# World extent visible on the canvas (meters)
WORLD_RANGE_X = 40  # meters forward/backward
WORLD_RANGE_Y = 40  # meters left/right

DOT_SIZE = 6  # px – waypoint / feature marker diameter
TARGET_SIZE = 8  # px – target location marker diameter


# || Coordinate transforms


class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


def world_to_canvas(point: Point) -> tuple[float, float]:
    """
    Convert a world-space point to logical canvas coordinates.

    World convention:  +X = forward (up on screen), +Y = left (left on screen)
    Canvas convention: origin at bottom-center; +canvas_x = right, +canvas_y = up

    Axis mapping:
        world +X (forward) → canvas -Y (up on screen, since screen Y is flipped)
        world +Y (left)    → canvas -X (left on screen)

    Both axes are scaled so WORLD_RANGE maps to the full canvas extent.
    """
    canvas_x = -point.y / WORLD_RANGE_Y * CANVAS_W  # left/right
    canvas_y = -point.x / WORLD_RANGE_X * CANVAS_H  # forward/back
    return canvas_x, canvas_y


def canvas_to_screen(
    cx: float, cy: float, screen_w: int, screen_h: int
) -> tuple[int, int]:
    """
    Convert logical canvas coordinates to screen pixel coordinates.

    The logical canvas is CANVAS_W × CANVAS_H units, with the car origin at
    (ORIGIN_X, ORIGIN_Y). This function:
      1. Shifts the point relative to the car origin.
      2. Scales to fill the current widget dimensions.

    Screen convention: origin top-left, +X right, +Y down.
    Note: canvas_y is already negative for "up", so adding it to ORIGIN_Y
    correctly moves points toward the top of the screen.
    """
    scale_x = screen_w / CANVAS_W
    scale_y = screen_h / CANVAS_H

    sx = (ORIGIN_X + cx) * scale_x
    sy = (ORIGIN_Y + cy) * scale_y
    return int(sx), int(sy)


# || Widget


class PathCanvas(QWidget):
    REFRESH_RATE = 10  # Hz

    def __init__(self, node: AP1ConsoleNode, parent=None):
        super().__init__(parent)
        self.node = node

        self.setMinimumSize(300, 300)
        self.setStyleSheet("background: black;")

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000 / self.REFRESH_RATE))

        # zoom and pan
        self._zoom = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._drag_start = None
        self._pan_start = None
        self.setMouseTracking(False)

    def world_to_screen(self, point: Point) -> tuple[int, int]:
        """
        Full pipeline: world space → logical canvas → screen pixels.
        Use this for all drawing operations.
        """
        effective_range_x = WORLD_RANGE_X * self._zoom
        effective_range_y = WORLD_RANGE_Y * self._zoom
        cx = -(point.y - self._pan_y) / effective_range_y * CANVAS_W
        cy = -(point.x - self._pan_x) / effective_range_x * CANVAS_H

        return canvas_to_screen(cx, cy, self.width(), self.height())

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        factor = 1.1 if delta < 0 else 0.9  # scroll up = zoom in
        self._zoom = max(
            0.1, min(10.0, self._zoom * factor)
        )  # min zoom is 10% max is 1000%
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_start = event.position()
            self._pan_start = (self._pan_x, self._pan_y)

    def mouseMoveEvent(self, event):
        if self._drag_start is not None:
            dx = event.position().x() - self._drag_start.x()
            dy = event.position().y() - self._drag_start.y()
            # convert screen delte -> world delta
            w, h = self.width(), self.height()
            world_range_x = WORLD_RANGE_X * self._zoom
            world_range_y = WORLD_RANGE_Y * self._zoom
            self._pan_x = self._pan_start[0] + (dy / h) * world_range_x
            self._pan_y = self._pan_start[1] + (dx / w) * world_range_y
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_start = None

    def _draw_car(self, painter: QPainter):
        """Draw a small cross/crosshair at the world origin representing the car."""
        px, py = self.world_to_screen(Point(0, 0))
        size = 8

        pen = QPen(YELLOW)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        # Cross
        painter.drawLine(px - size, py, px + size, py)
        painter.drawLine(px, py - size, px, py + size)

        # Circle around it
        painter.drawEllipse(px - size, py - size, size * 2, size * 2)

    #  Painting
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        painter.fillRect(0, 0, self.width(), self.height(), BLACK)

        self._draw_axes(painter)
        self._draw_path(painter)
        self._draw_features(painter)
        self._draw_lanes(painter)
        self._draw_car(painter)
        self._draw_legend(painter)

        painter.end()


    def _draw_legend(self, painter: QPainter):
        """Draw a color key in the bottom-right corner"""
        entries = [
            ("Waypoint", PURPLE, "ellipse"),
            ("Path", WHITE, "line"),
            ("Left lane", PURPLE, "line"),
            ("Right lane", BLUE, "line"),
            ("Stop sign", RED, "rect"),
            ("Car", YELLOW, "rect"),
        ]

        box_size = 10
        row_h = 18
        padding = 8
        legend_w = 120
        legend_h = len(entries) * row_h + padding * 2

        lx = self.width() - legend_w - padding 
        ly = self.height() - legend_h - padding 

        # Background
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(Qt.GlobalColor.black))
        painter.setOpacity(0.6)
        painter.drawRect(lx - 4, ly - 4, legend_w + 8, legend_h + 8)
        painter.setOpacity(1.0)

        for i, (label, color, shape) in enumerate(entries):
            y = ly + padding + i * row_h 
            sx = lx + 4 
            sy = y + row_h // 2

            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(color))

            if shape == "ellipse":
                r = box_size // 2
                painter.drawEllipse(sx, sy - r, box_size, box_size)
            elif shape == "rect":
                painter.drawRect(sx, sy - box_size // 2, box_size, box_size)
            elif shape == "line":
                pen = QPen(color)
                pen.setWidth(2)
                painter.setPen(pen)
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawLine(sx, sy, sx + box_size, sy)

            painter.setPen(QPen(WHITE))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawText(sx + box_size + 6, sy + 4, label)


    def _draw_axes(self, painter: QPainter):
        """Draw the X (forward) and Y (left) reference axes through the car origin, with labels."""
        w = self.width()
        h = self.height()

        pen = QPen(DIM)
        pen.setWidth(1)
        painter.setPen(pen)

        ox, oy = canvas_to_screen(0, 0, w, h)
        painter.drawLine(0, oy, w, oy)  # horizontal — left/right axis (+Y)
        painter.drawLine(ox, 0, ox, h)  # vertical   — forward axis    (+X)

        # Axis labels — drawn in dim gray near the positive ends of each axis
        painter.setPen(QPen(DIM))
        margin = 4  # px from edge / axis line

        # +X label: top of the vertical axis (forward = up)
        painter.drawText(ox + margin, margin + 12, "+x")

        # +Y label: left edge of the horizontal axis (left = left)
        painter.drawText(margin, oy - margin, "+y")

    def _draw_path(self, painter: QPainter):
        """Draw lines between consecutive waypoints, then waypoint dots on top."""
        waypoints = self.node.target_path[:]

        # Lines between consecutive waypoints
        pen = QPen(WHITE)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        for a, b in zip(waypoints, waypoints[1:]):
            x1, y1 = self.world_to_screen(a)
            x2, y2 = self.world_to_screen(b)
            painter.drawLine(x1, y1, x2, y2)

        # Dots at each waypoint (drawn after lines so they sit on top)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(GREEN))
        r = DOT_SIZE // 2

        for pt in waypoints:
            px, py = self.world_to_screen(pt)
            painter.drawEllipse(px - r, py - r, DOT_SIZE, DOT_SIZE)

    def _draw_features(self, painter: QPainter):
        """Draw detected features (stop signs, traffic lights, etc.) as colored squares."""
        painter.setPen(Qt.PenStyle.NoPen)
        r = DOT_SIZE // 2

        for entity in self.node.entities:
            x, y = entity.x, entity.y
            feature_type = DEFAULT_FEATURE_TYPE
            color = FEATURE_COLORS.get(feature_type, WHITE)
            painter.setBrush(QBrush(color))

            px, py = self.world_to_screen(Point(x, y))
            painter.drawRect(px - r, py - r, DOT_SIZE, DOT_SIZE)

    @DeprecationWarning
    def _draw_target(self, painter: QPainter):
        """Draw the navigation target location as a green circle."""
        if self.node.target_location is None:
            return

        tx, ty = self.node.target_location
        px, py = self.world_to_screen(Point(tx, ty))
        r = TARGET_SIZE // 2

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(GREEN))
        painter.drawEllipse(px - r, py - r, TARGET_SIZE, TARGET_SIZE)

    def _draw_lanes(self, painter: QPainter):
        """Draw lane boundaries as polylines — left in purple, right in blue."""
        if self.node.lane is None:
            return

        painter.setBrush(Qt.BrushStyle.NoBrush)

        for side, color in (
            (self.node.lane.left, PURPLE),
            (self.node.lane.right, BLUE),
        ):
            pen = QPen(color)
            pen.setWidth(2)
            painter.setPen(pen)

            for p1, p2 in zip(side, side[1:]):
                x1, y1 = self.world_to_screen(p1)
                x2, y2 = self.world_to_screen(p2)
                painter.drawLine(x1, y1, x2, y2)
