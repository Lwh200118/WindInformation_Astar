import imageio
import os
from PIL import Image

def create_gif(image_folder, output_path, duration_per_frame=1.0, delay=0.5):
    images = []
    current_time = 0.0
    while True:
        filename = os.path.join(image_folder, f"More_F0200F320200_New_City_10_{current_time:.1f}.png")
        if os.path.exists(filename):
            images.append(imageio.imread(filename))
        else:
            # 当找不到文件时，跳出循环
            break
        current_time += 1.0

    if images:
        # 添加延迟帧
        delay_frames = int(delay / duration_per_frame)
        new_images = []
        for img in images:
            new_images.append(img)
            # 通过重复当前图像添加延迟帧
            new_images.extend([img] * delay_frames)
        
        imageio.mimsave(output_path, new_images, duration=duration_per_frame)
        print(f"GIF saved to {output_path}")
    else:
        print("No images found to create a GIF.")

if __name__ == '__main__':
    image_folder = 'output_images'  # 图像存储的文件夹
    output_path = 'output_images/animation_More_F0200F320200_New_City1_1.0s.gif'  # GIF 保存路径
    create_gif(image_folder, output_path, duration_per_frame=1.8, delay=0.4)  # 设置每帧持续时间为0.5秒，延迟0.1秒