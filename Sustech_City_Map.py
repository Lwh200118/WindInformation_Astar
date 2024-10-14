import numpy as np
import trimesh
import matplotlib.pyplot as plt

def create_2d_grid(xmin, xmax, ymin, ymax, resolution):
    x = np.arange(xmin, xmax, resolution)
    y = np.arange(ymin, ymax, resolution)
    grid = np.meshgrid(x, y, indexing='ij')
    print(f"Grid X range: {x.min()} to {x.max()}")
    print(f"Grid Y range: {y.min()} to {y.max()}")
    return grid

def slice_stl_at_height(mesh, height):
    slice_mesh = mesh.section(plane_origin=[0, 0, height], plane_normal=[0, 0, 1])
    if slice_mesh is None:
        return None
    slice_2D, _ = slice_mesh.to_planar()
    return slice_2D

def plot_slice(slice_2D):
    fig, ax = plt.subplots()
    for entity in slice_2D.entities:
        if isinstance(entity, trimesh.path.entities.Line):
            start, end = entity.end_points
            start = slice_2D.vertices[start]
            end = slice_2D.vertices[end]
            ax.plot([start[0], end[0]], [start[1], end[1]], 'k-')
    ax.set_aspect('equal')
    plt.show()

def points_inside_mesh(mesh, grid, height):
    x, y = grid
    z = np.full_like(x, height)
    points = np.vstack([x.ravel(), y.ravel(), z.ravel()]).T
    print(f"Generated points range: X({x.min()} to {x.max()}), Y({y.min()} to {y.max()})")
    inside = mesh.contains(points)
    points_inside = points[inside]
    print(f"Points inside mesh: {points_inside.shape[0]} points")
    return points_inside

def plot_points(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:,0], points[:,1], points[:,2], s=1)
    plt.show()

def save_points_to_dat(points, filename):
    print(f"Saving points range: X({points[:,0].min()} to {points[:,0].max()}), Y({points[:,1].min()} to {points[:,1].max()})")
    np.savetxt(filename, points, fmt='%.6f', delimiter=' ', header='X Y Z')

def fill_missing_parts(points, resolution):
    filled_points = []
    for point in points:
        x, y, z = point
        neighbors = [
            [x + resolution, y, z],
            [x - resolution, y, z],
            [x, y + resolution, z],
            [x, y - resolution, z]
        ]
        for neighbor in neighbors:
            if not any(np.all(neighbor == p) for p in points):
                filled_points.append(neighbor)
    filled_points = np.array(filled_points)
    return np.vstack((points, filled_points))

# 设置计算域和分辨率
xmin, xmax, ymin, ymax = -1300, 1300, -1300, 1300
resolution = 1.25  # 提高分辨率

# 创建二维网格
grid = create_2d_grid(xmin, xmax, ymin, ymax, resolution)

# 读取STL文件
mesh = trimesh.load_mesh(r"E:\\gtian\\BlockMesh-LES-1600w-west\\sustech.stl")

# 截取固定高度的切面
height = 60.0
slice_2D = slice_stl_at_height(mesh, height)

if slice_2D:
    # 显示切面
    plot_slice(slice_2D)

    # 获取建筑物内部的点
    points_inside = points_inside_mesh(mesh, grid, height)

    # 填补缺失的部分
    filled_points = fill_missing_parts(points_inside, resolution)

    # 显示建筑物内部的点
    plot_points(filled_points)

    # 保存点到dat文件
    save_points_to_dat(filled_points, r"E:\\gtian\\BlockMesh-LES-1600w-west\\postProcessing\\surfaces\\3600\\Sustech_H_60.dat")
else:
    print("无法在该高度截取切面")