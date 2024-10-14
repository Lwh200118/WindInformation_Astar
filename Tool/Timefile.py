import os

def format_time_step_folders(directory):
    # 遍历目录中的所有文件夹
    for foldername in os.listdir(directory):
        folder_path = os.path.join(directory, foldername)
        if os.path.isdir(folder_path) and foldername.replace('.', '', 1).isdigit():
            # 提取时间步
            try:
                time_step = float(foldername)
            except ValueError:
                continue  # 跳过无法转换的文件夹名
            # 检查时间步是否在3600到3650之间
            if 3600 <= time_step <= 3655:
                # 格式化时间步为整数形式
                formatted_time_step = f"{int(time_step)}"
                # 构建新的文件夹名
                new_foldername = f"{formatted_time_step}"
                # 获取文件夹的完整路径
                new_folder_path = os.path.join(directory, new_foldername)
                # 重命名文件夹
                os.rename(folder_path, new_folder_path)
                print(f"Renamed: {folder_path} -> {new_folder_path}")

# 指定目录路径
directory_path = r"E:\gtian\BlockMesh-LES-1600w-west\postProcessing\surfaces"

# 调用函数格式化时间步文件夹
format_time_step_folders(directory_path)