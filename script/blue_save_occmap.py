#!/usr/bin/env python3
"""Save an OccupancyGrid map to the blue map files."""

import argparse
from pathlib import Path

import rclpy

from save_occmap import MapSaverNode


def main():
    script_path = Path(__file__).resolve()
    project_root = script_path.parent.parent
    default_output = project_root / "map" / "map_nav_blue"

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
