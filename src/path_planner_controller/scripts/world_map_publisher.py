#!/usr/bin/env python3
"""world_map_publisher

Hand-builds a nav_msgs/OccupancyGrid from the wall/pillar/crate geometry in
complex_world.world and continuously publishes it (1 Hz) on the "map" topic.
Purpose: let the path_planner_node (A*) + pure_pursuit_controller_node
pipeline be tested directly, without a real SLAM/map_server.

Why it publishes continuously instead of once: path_planner_node ignores map
messages until a /goal_pose arrives (need_plan_ == false), so the map has to
be sent periodically rather than once, so that a map message arriving AFTER
goal_pose triggers planning.

Usage:
    source /opt/ros/humble/setup.bash
    python3 world_map_publisher.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

RESOLUTION = 0.05
ORIGIN_X = -6.0
ORIGIN_Y = -5.0
WIDTH = 240   # 12m / 0.05
HEIGHT = 200  # 10m / 0.05

# (cx, cy, size_x, size_y) -- all static box walls in complex_world.world
WALLS = [
    # outer boundaries
    (0.0, 5.0, 12.24, 0.12),
    (0.0, -5.0, 12.24, 0.12),
    (6.0, 0.0, 0.12, 10.0),
    (-6.0, 0.0, 0.12, 10.0),
    # room dividers (x=-2 and x=2), with a 1m door gap in y=[-0.5,0.5]
    (-2.0, 2.75, 0.12, 4.5),
    (-2.0, -2.75, 0.12, 4.5),
    (2.0, 2.75, 0.12, 4.5),
    (2.0, -2.75, 0.12, 4.5),
    # maze baffles (left room)
    (-4.9, 3.2, 2.2, 0.12),
    (-3.1, 1.0, 2.2, 0.12),
    (-4.9, -1.2, 2.2, 0.12),
    (-3.1, -3.4, 2.2, 0.12),
    # slalom baffles (right room)
    (3.1, 3.0, 2.2, 0.12),
    (4.9, 0.5, 2.2, 0.12),
    (3.1, -2.5, 2.2, 0.12),
    # crates in the goal corner (rough box approximation, rotation ignored)
    (5.3, 4.0, 0.5, 0.5),
    (4.6, 4.4, 0.4, 0.4),
]

# (cx, cy, radius) -- scattered cylindrical pillars in the middle room
PILLARS = [
    (-0.8, 3.0, 0.3),
    (1.0, 2.0, 0.2),
    (-0.5, -1.5, 0.35),
    (0.8, -2.5, 0.25),
    (0.0, -3.8, 0.3),
]


def build_grid():
    data = [0] * (WIDTH * HEIGHT)
    for row in range(HEIGHT):
        y = ORIGIN_Y + (row + 0.5) * RESOLUTION
        for col in range(WIDTH):
            x = ORIGIN_X + (col + 0.5) * RESOLUTION
            occ = False
            for cx, cy, sx, sy in WALLS:
                if abs(x - cx) <= sx / 2.0 and abs(y - cy) <= sy / 2.0:
                    occ = True
                    break
            if not occ:
                for cx, cy, r in PILLARS:
                    if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
                        occ = True
                        break
            data[row * WIDTH + col] = 100 if occ else 0
    return data


class WorldMapPublisher(Node):
    def __init__(self):
        super().__init__('world_map_publisher')
        # RViz's Map display expects Transient Local (latched) publishing by default;
        # it didn't match with Volatile QoS. Also compatible with path_planner_node's
        # own (Volatile) map_sub_ (offered TRANSIENT_LOCAL >= requested VOLATILE is a
        # valid match).
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(OccupancyGrid, 'map', map_qos)
        self.grid_data = build_grid()
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info(
            f'complex_world map ready: {WIDTH}x{HEIGHT} cells, '
            f'{RESOLUTION} m/cell, publishing on "map" topic at 1 Hz.')

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = RESOLUTION
        msg.info.width = WIDTH
        msg.info.height = HEIGHT
        msg.info.origin.position.x = ORIGIN_X
        msg.info.origin.position.y = ORIGIN_Y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid_data
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = WorldMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
