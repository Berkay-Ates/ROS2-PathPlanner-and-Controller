#!/usr/bin/env python3
"""world_map_publisher

complex_world.world icindeki duvar/direk/sandik geometrisinden elle bir
nav_msgs/OccupancyGrid uretip "map" topic'ine surekli (1 Hz) yayinlar.
Amac: path_planner_node (A*) + pure_pursuit_controller_node pipeline'ini
gercek bir SLAM/map_server olmadan, dogrudan test edebilmek.

Surekli yayinlanmasinin sebebi: path_planner_node bir /goal_pose gelene
kadar map mesajlarini yok sayiyor (need_plan_ == false), bu yuzden map'i
tek seferlik degil periyodik gondermek gerekiyor ki goal_pose'dan SONRA
gelecek bir map mesaji planlamayi tetiklesin.

Kullanim:
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

# (cx, cy, size_x, size_y) -- complex_world.world'deki tum static kutu duvarlar
WALLS = [
    # dis sinirlar
    (0.0, 5.0, 12.24, 0.12),
    (0.0, -5.0, 12.24, 0.12),
    (6.0, 0.0, 0.12, 10.0),
    (-6.0, 0.0, 0.12, 10.0),
    # oda ayiricilar (x=-2 ve x=2), y=[-0.5,0.5] arasi 1m kapi bosluklu
    (-2.0, 2.75, 0.12, 4.5),
    (-2.0, -2.75, 0.12, 4.5),
    (2.0, 2.75, 0.12, 4.5),
    (2.0, -2.75, 0.12, 4.5),
    # labirent bariyerleri (sol oda)
    (-4.9, 3.2, 2.2, 0.12),
    (-3.1, 1.0, 2.2, 0.12),
    (-4.9, -1.2, 2.2, 0.12),
    (-3.1, -3.4, 2.2, 0.12),
    # slalom bariyerleri (sag oda)
    (3.1, 3.0, 2.2, 0.12),
    (4.9, 0.5, 2.2, 0.12),
    (3.1, -2.5, 2.2, 0.12),
    # hedef kosedeki sandiklar (kaba kutu yaklasimi, rotasyon ihmal edildi)
    (5.3, 4.0, 0.5, 0.5),
    (4.6, 4.4, 0.4, 0.4),
]

# (cx, cy, radius) -- orta odadaki dagitilmis silindir direkler
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
        # RViz'in Map display'i varsayilan olarak Transient Local (latched) yayin
        # bekliyor; Volatile QoS ile eslesmiyordu. path_planner_node'un kendi
        # (Volatile) map_sub_'i ile de uyumlu (offered TRANSIENT_LOCAL >= requested
        # VOLATILE gecerli bir eslesme).
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(OccupancyGrid, 'map', map_qos)
        self.grid_data = build_grid()
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info(
            f'complex_world haritasi hazir: {WIDTH}x{HEIGHT} hucre, '
            f'{RESOLUTION} m/hucre, "map" topic\'ine 1 Hz yayinlaniyor.')

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
