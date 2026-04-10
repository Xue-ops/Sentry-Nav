#!/usr/bin/env python3
import os
import yaml
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class MapSaverNode(Node):
    def __init__(self, topic_name: str, output_base: str):
        super().__init__('simple_map_saver')

        self.topic_name = topic_name
        self.output_base = str(Path(output_base).expanduser())
        self.got_map = False

        Path(self.output_base).parent.mkdir(parents=True, exist_ok=True)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub = self.create_subscription(
            OccupancyGrid,
            self.topic_name,
            self.map_callback,
            qos
        )

        self.get_logger().info(f"Waiting for map on topic: {self.topic_name}")
        self.get_logger().info(f"Will save to: {self.output_base}.pgm/.yaml")

    def map_callback(self, msg: OccupancyGrid):
        if self.got_map:
            return

        self.got_map = True

        try:
            self.save_map(msg)
            self.get_logger().info("Map saved successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")
        finally:
            self.destroy_node()
            rclpy.shutdown()

        rclpy.shutdown()

    def save_map(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        if width == 0 or height == 0:
            raise RuntimeError("Received empty map.")

        pgm_path = self.output_base + '.pgm'
        yaml_path = self.output_base + '.yaml'

        data = list(msg.data)

        with open(pgm_path, 'wb') as f:
            header = f"P5\n# CREATOR: simple_map_saver\n{width} {height}\n255\n"
            f.write(header.encode())

            for y in range(height - 1, -1, -1):
                row = bytearray()
                for x in range(width):
                    i = y * width + x
                    val = data[i]

                    if val == -1:
                        pixel = 205
                    elif val >= 50:
                        pixel = 0
                    else:
                        pixel = 254

                    row.append(pixel)
                f.write(row)

        yaml_data = {
            'image': os.path.basename(pgm_path),
            'mode': 'trinary',
            'resolution': float(resolution),
            'origin': [
                float(origin.position.x),
                float(origin.position.y),
                0.0
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }

        with open(yaml_path, 'w') as f:
            yaml.safe_dump(yaml_data, f, sort_keys=False)

        self.get_logger().info(f"Saved PGM:  {pgm_path}")
        self.get_logger().info(f"Saved YAML: {yaml_path}")


def main():
    script_path = Path(__file__).resolve()
    project_root = script_path.parent.parent
    default_output = project_root / "map" / "map_nav"

    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/map', help='OccupancyGrid topic name')
    parser.add_argument('--output', default=str(default_output), help='Output path without extension')
    args = parser.parse_args()

    rclpy.init()
    node = MapSaverNode(args.topic, args.output)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()