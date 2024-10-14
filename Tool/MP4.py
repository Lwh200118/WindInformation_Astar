from moviepy.editor import VideoFileClip

def gif_to_mp4(gif_path, output_path):
    # 读取 GIF 文件
    clip = VideoFileClip(gif_path)
    
    # 将 GIF 文件转换为 MP4 文件
    clip.write_videofile(output_path, codec="libx264")
    print(f"MP4 saved to {output_path}")

if __name__ == '__main__':
    gif_path = r"E:\UAV\研究计划\TEST\output_images\animation_More_F0200F320200_New_City1_1.0s.gif"  # GIF 文件路径
    output_path = 'animation_More_New_City_F0200F320200_1.0s.mp4'  # MP4 保存路径
    gif_to_mp4(gif_path, output_path)  