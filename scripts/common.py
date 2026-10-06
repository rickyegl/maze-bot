import json
import math
from pathlib import Path

DIRS = ((1, 0), (0, 1), (-1, 0), (0, -1))


def config_path(name):
    path = Path(__file__).resolve().parents[1] / 'config' / name
    if not path.is_file():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('maze_bot')) / 'config' / name
    return path


def config(name):
    return json.loads(config_path(name).read_text())


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def step(cell, d):
    return cell[0] + d[0], cell[1] + d[1]


def limit(v, w, drive):
    if not (math.isfinite(v) and math.isfinite(w)):
        return 0.0, 0.0
    wheel = drive['max_wheel_rpm'] * math.pi / 30 * drive['wheel_radius']
    side = abs(v) + abs(w) * drive['wheel_separation'] / 2
    k = max(1.0, abs(v) / drive['max_linear'], abs(w) / drive['max_angular'], side / wheel)
    return v / k, w / k


def run(node_type):
    import rclpy
    rclpy.init()
    node = node_type()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
