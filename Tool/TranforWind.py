import os
import shutil

def convert_raw_to_dat(root_dir):
    """
    将指定目录及其子目录中的所有 U_0_mySurfaces_60.raw 文件转换为 U_0_mySurfaces_60.dat 文件，
    并删除 .dat 文件中的前两行。同时删除其余文件。
    """
    for subdir, _, files in os.walk(root_dir):
        for file in files:
            if file == 'U_0_mySurfaces_60.raw':
                raw_file_path = os.path.join(subdir, file)
                dat_file_path = raw_file_path.replace('.raw', '.dat')
                
                # 读取 .raw 文件内容
                with open(raw_file_path, 'r') as raw_file:
                    lines = raw_file.readlines()
                
                # 删除前两行
                lines = lines[2:]
                
                # 将修改后的内容写入 .dat 文件
                with open(dat_file_path, 'w') as dat_file:
                    dat_file.writelines(lines)
                
                print(f"已将 {raw_file_path} 转换为 {dat_file_path}")
                
                # 删除其余文件
                for remaining_file in files:
                    remaining_file_path = os.path.join(subdir, remaining_file)
                    if remaining_file_path != raw_file_path and remaining_file_path != dat_file_path:
                        os.remove(remaining_file_path)
                        print(f"已删除 {remaining_file_path}")

if __name__ == "__main__":
    # 指定根目录路径
    root_directory = r"E:\gtian\BlockMesh-LES-1600w-west\postProcessing\5more"  # 请将此路径替换为实际路径
    convert_raw_to_dat(root_directory)