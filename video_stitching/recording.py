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
    # 设置RTSP传输参数
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
    
    # 打开视频流时增加缓冲设置
    cap = cv2.VideoCapture(rtsp_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)  # 设置缓冲区大小
    
    if not cap.isOpened():
        print(f"无法连接到 {rtsp_url}")
        return

    # 获取视频参数
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0:
        fps = 25
    
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # 设置视频输出
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))

    start_time = time.time()
    retry_count = 0
    max_retries = 3  # 最大重试次数

    while True:
        ret, frame = cap.read()
        
        # 处理帧获取失败的情况
        if not ret:
            retry_count += 1
            print(f"Frame read failed for {rtsp_url}, retry {retry_count}/{max_retries}")
            
            if retry_count >= max_retries:
                # 重新建立连接
                cap.release()
                time.sleep(1)  # 等待1秒
                cap = cv2.VideoCapture(rtsp_url)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
                retry_count = 0
            
            continue

        # 成功获取到帧，重置重试计数
        retry_count = 0
        
        # 写入帧
        out.write(frame)

        # 检查录制时间
        if time.time() - start_time >= RECORD_TIME:
            break

        # 添加短暂延时，避免CPU占用过高
        time.sleep(0.001)

    # 释放资源
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

