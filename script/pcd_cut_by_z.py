#!/usr/bin/env python3
import open3d as o3d
import numpy as np

# ========= 直接写死 =========
INPUT_PCD = "/home/xli/MT20260322_143851-Cloud_Opt.pcd"
OUTPUT_PCD = "/home/xli/0322_cut_up.pcd"
Z_THRESHOLD = 0
# ===========================

pcd = o3d.io.read_point_cloud(INPUT_PCD)
points = np.asarray(pcd.points)

if len(points) == 0:
    print("Point cloud is empty.")
    exit(1)

# 保留 z <= Z_THRESHOLD 的点
mask = points[:, 2] <= Z_THRESHOLD

filtered_pcd = o3d.geometry.PointCloud()
filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])

# 保留颜色
if pcd.has_colors():
    colors = np.asarray(pcd.colors)
    filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])

# 保留法向量
if pcd.has_normals():
    normals = np.asarray(pcd.normals)
    filtered_pcd.normals = o3d.utility.Vector3dVector(normals[mask])

o3d.io.write_point_cloud(OUTPUT_PCD, filtered_pcd)

print("Done.")
print(f"Input:  {INPUT_PCD}")
print(f"Output: {OUTPUT_PCD}")
print(f"Z threshold: {Z_THRESHOLD}")
print(f"Original points: {len(points)}")
print(f"Remaining points: {np.sum(mask)}")
print(f"Removed points: {len(points) - np.sum(mask)}")