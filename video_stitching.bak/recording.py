import cv2
import threading
import time
import os

# RTSP URL 列表
RTSP_URLS = [
    'rtsp://admin:xsznaf888@192.168.1.64/LiveMedia/ch1/Media1',
    'rtsp://admin:xsznaf888@192.168.1.65/LiveMedia/ch1/Media1',
    'rtsp://admin:xsznaf888@192.168.1.66/LiveMedia/ch1/Media1',
    'rtsp://admin:xsznaf888@192.168.1.67/LiveMedia/ch1/Media1',
    'rtsp://admin:xsznaf888@192.168.1.68/LiveMedia/ch1/Media1',
    'rtsp://admin:xsznaf888@192.168.1.69/LiveMedia/ch1/Media1'
]

# 每个视频流保存的文件名
output_files = [
    'camera_ff_1.avi',
    'camera_bb_2.avi',
    'camera_bl_3.avi',
    'camera_br_4.avi',
    'camera_fr_5.avi',
    'camera_fl_6.avi'
]

# 视频录制时长（秒）
RECORD_TIME = 10

def record_video(rtsp_url, output_file):
    # 打开视频流
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print(f"无法连接到 {rtsp_url}")
        return

    # 获取视频流的帧率和分辨率
    fps = cap.get(cv2.CAP_PROP_FPS)
    # 如果获取失败则设置默认帧率
    if fps == 0:
        fps = 25
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # 设置视频输出格式
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 使用XVID编码器
    out = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))

    # 录制开始时间
    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 写帧到输出文件
        out.write(frame)

        # 检查录制时间是否到达
        if time.time() - start_time >= RECORD_TIME:
            break

    # 释放视频流和输出资源
    cap.release()
    out.release()
    print(f"已保存视频文件: {output_file}")

if __name__ == '__main__':
    threads = []
    
    # 启动线程进行视频录制
    for i, rtsp_url in enumerate(RTSP_URLS):
        output_file = output_files[i]
        thread = threading.Thread(target=record_video, args=(rtsp_url, output_file))
        threads.append(thread)
        thread.start()

    # 等待所有线程结束
    for thread in threads:
        thread.join()

    print("所有视频录制完成。")

