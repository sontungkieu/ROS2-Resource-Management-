import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import os
import json

def find_pid_by_name(target_name):
    """Tìm PID của process dựa trên tên file chạy (ví dụ: heavy_node)"""
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            # Kiểm tra trong dòng lệnh chạy (cmdline) có chứa tên file không
            if proc.info['cmdline'] and any(target_name in s for s in proc.info['cmdline']):
                return proc.info['pid']
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return None

class CpuMonitorNode(Node):
    def __init__(self):
        super().__init__('cpu_monitor_node')
        
        # Publisher này cực kỳ quan trọng: Gửi data cho AI
        self.publisher_ = self.create_publisher(String, '/system_stats', 10)
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.target_name = "heavy_node"
        self.process = None
        
        # Thử tìm ngay lúc khởi động
        self.find_process()

    def find_process(self):
        pid = find_pid_by_name(self.target_name)
        if pid:
            self.process = psutil.Process(pid)
            self.get_logger().info(f'✅ Found {self.target_name} with PID: {pid}')
        else:
            self.get_logger().warn(f'⚠️ Waiting for {self.target_name} to start...')

    def timer_callback(self):
        # 1. Lấy CPU hệ thống
        sys_cpu = psutil.cpu_percent(interval=None)
        
        proc_cpu = 0.0
        proc_mem_mb = 0.0

        # 2. Lấy CPU của Heavy Node (nếu đã tìm thấy)
        if self.process:
            try:
                # Chia cho số core để ra % thực tế
                proc_cpu = self.process.cpu_percent(interval=None) / psutil.cpu_count()
                proc_mem_mb = self.process.memory_info().rss / (1024 * 1024)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                self.get_logger().error("Process lost! Searching again...")
                self.process = None
        else:
            # Nếu chưa có thì đi tìm lại
            self.find_process()

        # 3. ĐÓNG GÓI DỮ LIỆU (Phần bạn hỏi - Giữ nguyên!)
        stats_data = {
            "system_cpu_percent": sys_cpu,
            "target_node_cpu_percent": proc_cpu,
            "target_node_mem_mb": proc_mem_mb
        }
        
        # 4. Gửi đi
        msg = String()
        msg.data = json.dumps(stats_data)
        self.publisher_.publish(msg)
        
        # Log gọn hơn để dễ nhìn
        if self.process:
             self.get_logger().info(f'Pub: Sys={sys_cpu}% | Target={proc_cpu:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    node = CpuMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()