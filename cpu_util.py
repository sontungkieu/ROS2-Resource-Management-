import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import os
import json

def find_pid_by_name(target_name):
    # Hàm đi quét toàn bộ process trong OS để tìm tên
    for proc in psutil.process_iter(['pid', 'cmdline']):
        try:
            # Check xem dòng lệnh chạy có chứa tên file python không
            if proc.info['cmdline'] and any(target_name in s for s in proc.info['cmdline']):
                return proc.info['pid']
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return None

class CpuMonitorNode(Node):
    def __init__(self):
        super().__init__('cpu_monitor_node')
        
        # Publisher gửi data ra cho AI xử lý
        # Data dạng JSON string cho dễ đóng gói
        self.publisher_ = self.create_publisher(String, '/system_stats', 10)
        
        # Timer chạy mỗi 1 giây (1.0s) để lấy mẫu
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Giả sử chúng ta muốn theo dõi một process cụ thể
        # Trong thực tế, bạn sẽ tìm PID của node "tính toán nặng" 
        # Ở đây tôi lấy ví dụ là chính node hiện tại (os.getpid())
        self.target_pid = os.getpid() 
        self.process = psutil.Process(self.target_pid)
        
        self.get_logger().info(f'Monitor Node Started. Tracking PID: {self.target_pid}')

    def timer_callback(self):
        # 1. Lấy CPU usage toàn hệ thống (System-wide)
        # interval=None nghĩa là lấy số liệu tức thời không chặn (non-blocking)
        sys_cpu = psutil.cpu_percent(interval=None)
        
        # 2. Lấy CPU usage của tiến trình đích (Process-specific)
        try:
            # Lấy % CPU của tiến trình. /os.cpu_count() để chuẩn hóa về thang 100%
            # Vì trên máy đa nhân, 1 process full core có thể là 100%, 
            # nhưng psutil có thể trả về >100% nếu không chia.
            proc_cpu = self.process.cpu_percent(interval=None) / psutil.cpu_count()
            
            # Lấy lượng RAM đang dùng (RSS - Resident Set Size)
            proc_mem_info = self.process.memory_info()
            proc_mem_mb = proc_mem_info.rss / (1024 * 1024) # Đổi ra MB
            
        except psutil.NoSuchProcess:
            self.get_logger().error("Process not found!")
            proc_cpu = 0.0
            proc_mem_mb = 0.0

        # 3. Đóng gói dữ liệu (State) để gửi cho AI
        stats_data = {
            "system_cpu_percent": sys_cpu,
            "target_node_cpu_percent": proc_cpu,
            "target_node_mem_mb": proc_mem_mb
        }
        
        # 4. Publish message
        msg = String()
        msg.data = json.dumps(stats_data)
        self.publisher_.publish(msg)
        
        # Log ra màn hình để debug
        self.get_logger().info(f'Publishing stats: {msg.data}')

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