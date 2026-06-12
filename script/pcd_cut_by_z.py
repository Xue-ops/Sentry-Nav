#!/usr/bin/env python3
import struct
import sys
import os

# ========= 直接写死 =========
INPUT_PCD = "/home/wele/ros_ws/src/Mapping-and-Relocalization/small_point_lio/pcd/scan_baselink.pcd"
OUTPUT_PCD = "/home/wele/ros_ws/src/Mapping-and-Relocalization/small_point_lio/pcd/scan_baselink_cut.pcd"
Z_THRESHOLD = 1.0
# ===========================


def read_header(f):
    header_lines = []
    header = {}

    while True:
        line = f.readline()
        if not line:
            raise RuntimeError("Invalid PCD file: cannot find DATA line.")

        header_lines.append(line)
        line_str = line.decode(errors="ignore").strip()

        if line_str and not line_str.startswith("#"):
            parts = line_str.split()
            key = parts[0]
            values = parts[1:]
            header[key] = values

        if line_str.startswith("DATA"):
            data_type = line_str.split()[1]
            return header_lines, header, data_type


def update_header(header_lines, new_points):
    new_header = []

    for raw in header_lines:
        line = raw.decode(errors="ignore")

        if line.startswith("WIDTH"):
            new_header.append(f"WIDTH {new_points}\n".encode())
        elif line.startswith("HEIGHT"):
            new_header.append(b"HEIGHT 1\n")
        elif line.startswith("POINTS"):
            new_header.append(f"POINTS {new_points}\n".encode())
        else:
            new_header.append(raw)

    return new_header


def cut_ascii(input_path, output_path, z_threshold):
    with open(input_path, "rb") as f:
        header_lines, header, data_type = read_header(f)
        point_lines = f.readlines()

    fields = header.get("FIELDS", [])
    if "z" not in fields:
        raise RuntimeError("PCD has no z field.")

    z_index = fields.index("z")

    kept = []
    original_points = 0

    for raw in point_lines:
        line = raw.decode(errors="ignore").strip()
        if not line:
            continue

        parts = line.split()
        if len(parts) <= z_index:
            continue

        original_points += 1
        z = float(parts[z_index])

        if z <= z_threshold:
            kept.append(raw)

    new_header = update_header(header_lines, len(kept))

    with open(output_path, "wb") as f:
        f.writelines(new_header)
        f.writelines(kept)

    return original_points, len(kept)


def get_struct_format(types, sizes, counts):
    fmt = "<"

    for t, s, c in zip(types, sizes, counts):
        s = int(s)
        c = int(c)

        if t == "F" and s == 4:
            code = "f"
        elif t == "F" and s == 8:
            code = "d"
        elif t == "U" and s == 1:
            code = "B"
        elif t == "U" and s == 2:
            code = "H"
        elif t == "U" and s == 4:
            code = "I"
        elif t == "I" and s == 1:
            code = "b"
        elif t == "I" and s == 2:
            code = "h"
        elif t == "I" and s == 4:
            code = "i"
        else:
            raise RuntimeError(f"Unsupported PCD field: TYPE={t}, SIZE={s}")

        fmt += code * c

    return fmt


def get_unpack_index_of_field(fields, counts, target_field):
    index = 0

    for field, count in zip(fields, counts):
        if field == target_field:
            return index
        index += int(count)

    raise RuntimeError(f"Field {target_field} not found.")


def cut_binary(input_path, output_path, z_threshold):
    with open(input_path, "rb") as f:
        header_lines, header, data_type = read_header(f)
        binary_data = f.read()

    fields = header.get("FIELDS", [])
    sizes = header.get("SIZE", [])
    types = header.get("TYPE", [])
    counts = header.get("COUNT", ["1"] * len(fields))

    if "z" not in fields:
        raise RuntimeError("PCD has no z field.")

    fmt = get_struct_format(types, sizes, counts)
    point_step = struct.calcsize(fmt)
    z_unpack_index = get_unpack_index_of_field(fields, counts, "z")

    total_points = len(binary_data) // point_step
    kept_data = bytearray()

    for i in range(total_points):
        start = i * point_step
        end = start + point_step
        point_raw = binary_data[start:end]

        values = struct.unpack(fmt, point_raw)
        z = values[z_unpack_index]

        if z <= z_threshold:
            kept_data.extend(point_raw)

    kept_points = len(kept_data) // point_step
    new_header = update_header(header_lines, kept_points)

    with open(output_path, "wb") as f:
        f.writelines(new_header)
        f.write(kept_data)

    return total_points, kept_points


def main():
    input_path = INPUT_PCD
    output_path = OUTPUT_PCD
    z_threshold = Z_THRESHOLD

    if len(sys.argv) >= 2:
        input_path = sys.argv[1]
    if len(sys.argv) >= 3:
        output_path = sys.argv[2]
    if len(sys.argv) >= 4:
        z_threshold = float(sys.argv[3])

    if not os.path.exists(input_path):
        raise RuntimeError(f"Input file does not exist: {input_path}")

    with open(input_path, "rb") as f:
        _, _, data_type = read_header(f)

    if data_type == "ascii":
        original_points, kept_points = cut_ascii(input_path, output_path, z_threshold)
    elif data_type == "binary":
        original_points, kept_points = cut_binary(input_path, output_path, z_threshold)
    elif data_type == "binary_compressed":
        raise RuntimeError(
            "This PCD is DATA binary_compressed. "
            "Pure Python cannot easily handle it. Convert it to ascii or binary first."
        )
    else:
        raise RuntimeError(f"Unknown PCD DATA type: {data_type}")

    print("Done.")
    print(f"Input:  {input_path}")
    print(f"Output: {output_path}")
    print(f"Z threshold: {z_threshold}")
    print(f"PCD data type: {data_type}")
    print(f"Original points: {original_points}")
    print(f"Remaining points: {kept_points}")
    print(f"Removed points: {original_points - kept_points}")


if __name__ == "__main__":
    main()