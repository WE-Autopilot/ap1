from rclpy.node import Node
from geometry_msgs.msg import Point 
from std_msgs.msg import String
from ap1_msgs.msg import TargetPathStamped, SpeedProfileStamped, FloatStamped, FloatStamped, LaneBoundaries, EntityState, EntityStateArray

import xml.etree.ElementTree as ET
from std_msgs.msg import String

class AP1ConsoleNode(Node):
    def __init__(self):
        super().__init__('ap1_console')

        # publishers
        self.speed_pub = self.create_publisher(FloatStamped, '/ap1/control/target_speed', 1)
        self.target_location_pub = self.create_publisher(Point, '/ap1/control/target_location', 1)

        # subscribers
        self.speed_sub = self.create_subscription(FloatStamped, '/ap1/actuation/speed', self.speed_callback, 1)
        self.turn_angle_sub = self.create_subscription(FloatStamped, '/ap1/control/turn_angle', self.turn_angle_callback, 1)
        self.current_motor_power_sub = self.create_subscription(FloatStamped, '/ap1/control/motor_power', self.motor_power_callback, 1)
        self.brake_sub = self.create_subscription(FloatStamped, '/ap1/control/brake', self.brake_callback, 1)
        self.path_sub = self.create_subscription(TargetPathStamped, '/ap1/planning/target_path', self.target_path_callback, 1)
        self.speed_profile = self.create_subscription(SpeedProfileStamped, '/ap1/planning/speed_profile', self.speed_profile_callback, 1)
        self.lane_sub = self.create_subscription(LaneBoundaries, '/ap1/mapping/lanes', self.lane_callback, 1)
        self.entities_sub = self.create_subscription(EntityStateArray, '/ap1/mapping/entities', self.entities_callback, 1)
        self.planning_state_sub = self.create_subscription(String, '/ap1/planning/state', self.planning_state_callback, 1)

        # Fields
        self.current_speed = 0.0 # m
        self.target_speed = 0.0 # m/s
        self.motor_power = 0.0 # [0, 1]
        self.brake = 0.0 # [0, 1]
        self.target_location = (0.0, 0.0) # m
        self.current_turn_angle = 0.0 # rads
        self.target_path = [] # waypoints
        self.speed_profile = [] # m/s
        self.lane: LaneBoundaries = None
        self.entities = []
        self.planning_state: str = "N/A"

    # TODO: replace all with anonymous funcs
    def brake_callback(self, msg):
        self.brake = msg.value

    def planning_state_callback(self, msg: String):
        self.planning_state = msg.data

    def entities_callback(self, msg: EntityStateArray):
        self.entities = msg.entities

    def speed_callback(self, msg: FloatStamped):
        self.current_speed = msg.value

    def speed_profile_callback(self, msg: SpeedProfileStamped):
        self.speed_profile = msg.speeds

    def turn_angle_callback(self, msg: FloatStamped):
        self.current_turn_angle = msg.value
    
    def motor_power_callback(self, msg: FloatStamped):
        self.motor_power = msg.value

    def target_path_callback(self, msg: TargetPathStamped):
        self.target_path = msg.path

    def lane_callback(self, msg: LaneBoundaries):
        self.lane = msg
    
    def set_target_speed(self, speed: float):
        self.target_speed = speed

        # assemble msg
        msg = FloatStamped()
        msg.value = speed
        msg.header.stamp = self.get_clock().now().to_msg()

        # send out msg
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Sent target speed: {speed}')

    def set_target_location(self, x: float, y: float):
        self.target_location = (x, y)
        msg = Point()
        msg.x = x
        msg.y = y 
        msg.z = 0.0
        self.target_location_pub.publish(msg)
        self.get_logger().info(f'Sent target location: ({x}, {y})')

    def xml_callback(self, msg: String):
        self.features.clear() # clear existing features
        root = ET.fromstring(msg.data)
        for feature in root.findall('Feature'): # parse features
            feature_type = feature.find('Type').text
            pos = feature.find('Position')
            x = float(pos.find('X').text)
            y = float(pos.find('Y').text)
            if feature_type in ['stop_sign', 'stop_line', 'traffic_light', 'yield_sign']: 
                self.features.append((feature_type, x, y)) 
            else: 
                self.features.append(('unknown', x, y))
        self.get_logger().info(f"Parsed {len(self.features)} features from XML")

# TEMPORARY
if __name__ == '__main__':
    import rclpy

    rclpy.init()
    node = AP1ConsoleNode()

    try:
        node.set_target_speed(2)
    finally:
        # teardown
        node.destroy_node()
        rclpy.shutdown()
    
