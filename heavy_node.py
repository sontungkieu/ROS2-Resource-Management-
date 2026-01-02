import rclpy
from rclpy.node import Node
import time
import math
import os

class HeavyNode(Node):
    def __init__(self):
        super().__init__('heavy_computation_node')
        
        # Khai báo tham số điều khiển: load_level (0.0 đến 1.0)
        # 1.0 = Chạy hết công suất, 0.1 = Chạy cầm chừng
        self.declare_parameter('load_level', 1.0)
        
        self.timer = self.create_timer(0.1, self.work_loop)
        self.get_logger().info(f'Heavy Node Started with PID: {os.getpid()}')

    def work_loop(self):
        # 1. Đọc tham số hiện tại từ hệ thống (do Manager chỉnh)
        load_level = self.get_parameter('load_level').get_parameter_value().double_value
        
        # 2. Logic "Đốt CPU"
        # Nếu load_level cao -> tính toán nhiều. Thấp -> ngủ nhiều.
        start_time = time.time()
        
        # Chu kỳ làm việc 100ms
        interval = 0.1 
        busy_time = interval * load_level
        
        # Busy Loop (Giả lập tính toán nặng)
        while (time.time() - start_time) < busy_time:
            # Phép tính vô nghĩa để tốn CPU
            _ = [math.sqrt(x) for x in range(1, 1000)]
            
        # Thời gian còn lại thì ngủ (nhường CPU cho tiến trình khác)
        remaining = interval - (time.time() - start_time)
        if remaining > 0:
            time.sleep(remaining)

        # Log nhẹ để biết đang chạy ở mức nào
        # self.get_logger().info(f'Running at load: {load_level*100}%')

def main(args=None):
    rclpy.init(args=args)
    node = HeavyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()