#!/usr/bin/env python3
"""Transform a lidar-frame PCD map into the base_link frame.

The transform argument is interpreted as base_link -> lidar, i.e. the lidar
pose expressed in base_link:

    point_base = R_base_lidar * point_lidar + t_base_lidar

Use --inverse if the numbers you have are lidar -> base_link instead.
"""

import argparse
import io
import math
from pathlib import Path

import numpy as np


TYPE_MAP = {
    ("F", 4): np.float32,
    ("F", 8): np.float64,
    ("I", 1): np.int8,
    ("I", 2): np.int16,
    ("I", 4): np.int32,
    ("I", 8): np.int64,
    ("U", 1): np.uint8,
    ("U", 2): np.uint16,
    ("U", 4): np.uint32,
    ("U", 8): np.uint64,
}

DEFAULT_INPUT_PCD = (
    "/home/xli/catkin_ws/src/mapping_relocalization/small_point_lio/pcd/scan.pcd"
)
DEFAULT_OUTPUT_PCD = (
    "/home/xli/catkin_ws/src/mapping_relocalization/small_point_lio/pcd/scan_baselink.pcd"
)
DEFAULT_TF_BASELINK_LIDAR = [
    0.0025, -0.13509, 0.29325, 0.0, 0.382684, -0.923879, 0.0
]


def parse_header(raw):
    offset = 0
    lines = []

    while True:
        next_offset = raw.find(b"\n", offset)
        if next_offset < 0:
            raise ValueError("Invalid PCD: missing DATA line")

        line_bytes = raw[offset:next_offset]
        offset = next_offset + 1
        line = line_bytes.decode("ascii").strip()
        lines.append(line)

        if line.upper().startswith("DATA "):
            return lines, offset


def header_value(lines, key, default=None):
    key = key.upper()
    for line in lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if parts and parts[0].upper() == key:
            return parts[1:]
    return default


def build_dtype(lines):
    fields = header_value(lines, "FIELDS")
    sizes = [int(v) for v in header_value(lines, "SIZE")]
    types = header_value(lines, "TYPE")
    counts = [int(v) for v in header_value(lines, "COUNT", ["1"] * len(fields))]

    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("Invalid PCD header: FIELDS/SIZE/TYPE/COUNT lengths differ")

    dtype_fields = []
    for name, size, type_name, count in zip(fields, sizes, types, counts):
        np_type = TYPE_MAP.get((type_name.upper(), size))
        if np_type is None:
            raise ValueError(f"Unsupported PCD field type: {type_name} size {size}")
        if count == 1:
            dtype_fields.append((name, np_type))
        else:
            dtype_fields.append((name, np_type, (count,)))

    return np.dtype(dtype_fields)


def point_count(lines):
    points = header_value(lines, "POINTS")
    if points:
        return int(points[0])

    width = int(header_value(lines, "WIDTH", ["0"])[0])
    height = int(header_value(lines, "HEIGHT", ["1"])[0])
    return width * height


def read_pcd(path):
    raw = Path(path).read_bytes()
    lines, data_offset = parse_header(raw)
    data_type = header_value(lines, "DATA")[0].lower()
    dtype = build_dtype(lines)
    count = point_count(lines)
    body = raw[data_offset:]

    if data_type == "ascii":
        cloud = np.loadtxt(io.BytesIO(body), dtype=dtype, ndmin=1)
    elif data_type == "binary":
        cloud = np.frombuffer(body, dtype=dtype, count=count).copy()
    elif data_type == "binary_compressed":
        raise ValueError("binary_compressed PCD is not supported by this script")
    else:
        raise ValueError(f"Unsupported PCD DATA type: {data_type}")

    if not {"x", "y", "z"}.issubset(cloud.dtype.names or []):
        raise ValueError("PCD must contain x, y, z fields")

    return lines, cloud, data_type


def quaternion_to_rotation(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("Quaternion norm is zero")

    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def transform_cloud(cloud, tf_values, inverse):
    x, y, z, qx, qy, qz, qw = tf_values
    rotation = quaternion_to_rotation(qx, qy, qz, qw)
    translation = np.array([x, y, z], dtype=np.float64)

    if inverse:
        rotation = rotation.T
        translation = -rotation @ translation

    points = np.column_stack(
        (
            cloud["x"].astype(np.float64),
            cloud["y"].astype(np.float64),
            cloud["z"].astype(np.float64),
        )
    )
    transformed = points @ rotation.T + translation

    cloud["x"] = transformed[:, 0].astype(cloud["x"].dtype, copy=False)
    cloud["y"] = transformed[:, 1].astype(cloud["y"].dtype, copy=False)
    cloud["z"] = transformed[:, 2].astype(cloud["z"].dtype, copy=False)


def write_pcd(path, lines, cloud, data_type):
    header = "\n".join(lines) + "\n"
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if data_type == "ascii":
        with output_path.open("w", encoding="ascii") as f:
            f.write(header)
            for row in cloud:
                values = []
                for name in cloud.dtype.names:
                    value = row[name]
                    if np.isscalar(value):
                        values.append(str(value.item()))
                    else:
                        values.extend(str(v.item()) for v in value)
                f.write(" ".join(values) + "\n")
    else:
        with output_path.open("wb") as f:
            f.write(header.encode("ascii"))
            f.write(cloud.tobytes())


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert a lidar-recorded PCD map into base_link coordinates."
    )
    parser.add_argument(
        "input_pcd",
        nargs="?",
        default=DEFAULT_INPUT_PCD,
        help=f"Input PCD whose x/y/z are in the lidar frame (default: {DEFAULT_INPUT_PCD})",
    )
    parser.add_argument(
        "output_pcd",
        nargs="?",
        default=DEFAULT_OUTPUT_PCD,
        help=f"Output PCD whose x/y/z are in base_link (default: {DEFAULT_OUTPUT_PCD})",
    )
    parser.add_argument(
        "--tf",
        nargs=7,
        type=float,
        default=DEFAULT_TF_BASELINK_LIDAR,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help=(
            "base_link -> lidar transform: x y z qx qy qz qw "
            f"(default: {' '.join(str(v) for v in DEFAULT_TF_BASELINK_LIDAR)})"
        ),
    )
    parser.add_argument(
        "--inverse",
        action="store_true",
        help="Treat --tf as lidar -> base_link and invert it before applying",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    lines, cloud, data_type = read_pcd(args.input_pcd)
    original_count = len(cloud)

    transform_cloud(cloud, args.tf, args.inverse)
    write_pcd(args.output_pcd, lines, cloud, data_type)

    direction = "lidar -> base_link, inverted" if args.inverse else "base_link -> lidar"
    print("Done.")
    print(f"Input:  {args.input_pcd}")
    print(f"Output: {args.output_pcd}")
    print(f"DATA:   {data_type}")
    print(f"Points: {original_count}")
    print(f"TF:     {direction}")


if __name__ == "__main__":
    main()
