#!/usr/bin/env python3
"""Build the blue-side PCD files from scan.pcd."""

import argparse
import shutil
from pathlib import Path

from pcd_cut_by_z import Z_THRESHOLD, cut_ascii, cut_binary, read_header
from pcd_lidar_to_baselink import (
    DEFAULT_INPUT_PCD,
    DEFAULT_TF_BASELINK_LIDAR,
    read_pcd,
    transform_cloud,
    write_pcd,
)


TEAM = "blue"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Copy scan.pcd to scan_blue.pcd, transform it to base_link, then cut by z."
    )
    parser.add_argument(
        "input_pcd",
        nargs="?",
        default=DEFAULT_INPUT_PCD,
        help=f"Input scan.pcd (default: {DEFAULT_INPUT_PCD})",
    )
    parser.add_argument(
        "--z-threshold",
        type=float,
        default=Z_THRESHOLD,
        help=f"Keep points with z <= threshold (default: {Z_THRESHOLD})",
    )
    parser.add_argument(
        "--tf",
        nargs=7,
        type=float,
        default=DEFAULT_TF_BASELINK_LIDAR,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="base_link -> lidar transform used before cutting",
    )
    parser.add_argument(
        "--inverse",
        action="store_true",
        help="Treat --tf as lidar -> base_link and invert it before applying",
    )
    return parser.parse_args()


def cut_by_z(input_path, output_path, z_threshold):
    with open(input_path, "rb") as f:
        _, _, data_type = read_header(f)

    if data_type == "ascii":
        return cut_ascii(input_path, output_path, z_threshold), data_type
    if data_type == "binary":
        return cut_binary(input_path, output_path, z_threshold), data_type
    if data_type == "binary_compressed":
        raise RuntimeError(
            "This PCD is DATA binary_compressed. Convert it to ascii or binary first."
        )

    raise RuntimeError(f"Unknown PCD DATA type: {data_type}")


def main():
    args = parse_args()
    source_path = Path(args.input_pcd)
    if not source_path.exists():
        raise RuntimeError(f"Input file does not exist: {source_path}")

    output_dir = source_path.parent
    team_pcd = output_dir / f"scan_{TEAM}.pcd"
    baselink_pcd = output_dir / f"scan_{TEAM}_baselink.pcd"
    cut_pcd = output_dir / f"scan_{TEAM}_baselink_cut.pcd"

    shutil.copyfile(source_path, team_pcd)

    lines, cloud, data_type = read_pcd(team_pcd)
    transform_cloud(cloud, args.tf, args.inverse)
    write_pcd(baselink_pcd, lines, cloud, data_type)

    (original_points, kept_points), cut_data_type = cut_by_z(
        baselink_pcd, cut_pcd, args.z_threshold
    )

    print("Done.")
    print(f"Input:       {source_path}")
    print(f"Copied:      {team_pcd}")
    print(f"Base_link:   {baselink_pcd}")
    print(f"Cut output:  {cut_pcd}")
    print(f"Z threshold: {args.z_threshold}")
    print(f"PCD data:    {cut_data_type}")
    print(f"Original points:  {original_points}")
    print(f"Remaining points: {kept_points}")
    print(f"Removed points:   {original_points - kept_points}")


if __name__ == "__main__":
    main()
