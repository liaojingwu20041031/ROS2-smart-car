#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped # 可以根据实际需要修改话题类型
import cv2
import threading
import socket
import json
import time

class AIVisionNode(Node):
    def __init__(self):
        super().__init__('ai_vision_node')
        
        # --- 参数配置 ---
        self.declare_parameter('pc_ip', '192.168.137.1') 
        self.declare_parameter('video_udp_port', 5000)
        self.declare_parameter('result_udp_port', 5001)
        self.declare_parameter('camera_id', '/dev/video0')
        
        self.pc_ip = self.get_parameter('pc_ip').value
        self.video_udp_port = self.get_parameter('video_udp_port').value
        self.result_udp_port = self.get_parameter('result_udp_port').value
        self.camera_id = self.get_parameter('camera_id').value

        # --- ROS 2 发布者 ---
        # 根据实际业务逻辑，这里发给导航、抓取或状态机
        self.vision_result_pub = self.create_publisher(String, '/vision/result', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, '/vision/target_pose', 10)
        
        # --- UDP UDP 设置 ---
        self.result_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.result_socket.bind(('0.0.0.0', self.result_udp_port))
        
        # 标志位
        self.is_running = True

        # --- 启动线程 ---
        self.camera_thread = threading.Thread(target=self.camera_capture_and_stream_loop)
        self.camera_thread.daemon = True
        self.camera_thread.start()

        self.result_recv_thread = threading.Thread(target=self.receive_results_loop)
        self.result_recv_thread.daemon = True
        self.result_recv_thread.start()

        self.get_logger().info(f"AI Vision Node Started. Streaming to {self.pc_ip}:{self.video_udp_port}")
        self.get_logger().info(f"Listening for results on UDP port {self.result_udp_port}")

    def camera_capture_and_stream_loop(self):
        """
        相机采集与 H.264 UDP 推流线程
        避开 OpenCV 数据中转，彻底使用底层的 GStreamer 管道推流，极大降低延迟与卡顿
        使用 v4l2src 直接推给硬解码/软解码，之后 h264 编码发出
        """
        import subprocess
        
        self.get_logger().info(f"Starting native GStreamer pipeline for {self.camera_id}...")

        # 既然是 720P(1280x720) MJPEG 输入，直接交给 GST 处理再推流
        gst_cmd = [
            'gst-launch-1.0',
            'v4l2src', f'device={self.camera_id}', '!',
            'image/jpeg,width=1280,height=720,framerate=30/1', '!',
            'jpegdec', '!',
            'videoconvert', '!',
            'video/x-raw,format=I420', '!',
            'x264enc', 'tune=zerolatency', 'bitrate=2048', 'speed-preset=ultrafast', '!',
            'h264parse', '!',
            'rtph264pay', 'config-interval=1', 'pt=96', '!',
            'udpsink', f'host={self.pc_ip}', f'port={self.video_udp_port}', 'sync=false'
        ]

        try:
            self.gst_process = subprocess.Popen(gst_cmd)
        except Exception as e:
            self.get_logger().error(f"Failed to start GStreamer process: {e}")
            return

        # 挂起监控进程存活
        while self.is_running:
            if self.gst_process.poll() is not None:
                self.get_logger().warn("GStreamer pipeline exited unexpectedly.")
                break
            time.sleep(1.0)
            
        try:
            self.gst_process.terminate()
            self.gst_process.wait(timeout=2)
        except Exception:
            pass

    def receive_results_loop(self):
        """
        接收 PC 回传的视觉结果并发布为 ROS2 话题
        """
        self.result_socket.settimeout(1.0)
        while self.is_running:
            try:
                data, addr = self.result_socket.recvfrom(2048)
                result_str = data.decode('utf-8')
                
                # 假设 PC 收发的是 JSON 格式结果，如：
                # {"target_name": "apple", "x": 1.0, "y": 0.5, "z": 0.0, "status": "found"}
                result_json = json.loads(result_str)
                
                # 1. 发布原始 JSON 字符串供状态机使用
                msg = String()
                msg.data = result_str
                self.vision_result_pub.publish(msg)
                
                # 2. 如果包含坐标信息，发布 PoseStamped 给导航或机械臂抓取
                if "x" in result_json and "y" in result_json:
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "camera_link" # 根据 TF 树修改
                    pose_msg.pose.position.x = float(result_json["x"])
                    pose_msg.pose.position.y = float(result_json["y"])
                    pose_msg.pose.position.z = float(result_json.get("z", 0.0))
                    # 姿态默认给单位四元数
                    pose_msg.pose.orientation.w = 1.0
                    self.target_pose_pub.publish(pose_msg)
                    
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Error receiving vision result: {e}")

    def stop(self):
        self.is_running = False
        if self.camera_thread.is_alive():
            self.camera_thread.join()
        if self.result_recv_thread.is_alive():
            self.result_recv_thread.join()
        if self.result_socket:
            self.result_socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = AIVisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AI Vision Node...")
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
