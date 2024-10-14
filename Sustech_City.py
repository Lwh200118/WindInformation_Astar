import numpy as np
import matplotlib.pyplot as plt

def read_dat_file(filename):
    """
    读取 .dat 文件并返回点的坐标
    """
    data = np.loadtxt(filename, skiprows=1)  # 跳过文件头
    return data

def plot_2d_points(points, title="2D Points Plot"):
    """
    绘制二维点
    """
    fig, ax = plt.subplots()
    ax.scatter(points[:, 0], points[:, 1], s=1, c='blue', marker='o')
    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_xlim([-1300, 1300])
    ax.set_ylim([-1300, 1300])
    ax.set_aspect('equal')
    plt.show()

def main():
    # 读取 .dat 文件
    filename = r"E:\gtian\BlockMesh-LES-1600w-west\Sustech_H_60.dat"
    points = read_dat_file(filename)

    # 绘制二维点
    plot_2d_points(points, title="2D Points from .dat File")

if __name__ == "__main__":
    main()