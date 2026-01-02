import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import json

class ManagerNode(Node):
    def __init__(self):
        super().__init__('resource_manager_node')
        
        # 1. Subscribe để nghe ngóng tình hình
        self.subscription = self.create_subscription(
            String, '/system_stats', self.listener_callback, 10)
            
        # 2. Client để gửi lệnh điều khiển (Set Parameter)
        self.client = self.create_client(SetParameters, '/heavy_computation_node/set_parameters')
        
        self.get_logger().info('👮 Manager AI is watching...')
        self.current_load = 1.0 # Mặc định đang chạy 100%

    def set_load_level(self, level):
        """Hàm gửi lệnh set_parameter sang Heavy Node"""
        req = SetParameters.Request()
        
        # Tạo parameter object đúng chuẩn ROS 2
        param_value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=level)
        param = Parameter(name='load_level', value=param_value)
        req.parameters = [param]
        
        # Gọi service (Call async để không bị block)
        self.client.call_async(req)
        self.get_logger().warn(f'⚡ ACTION: Adjusting Load -> {level*100}%')

    def listener_callback(self, msg):
        # 1. Giải mã dữ liệu
        data = json.loads(msg.data)
        sys_cpu = data['system_cpu_percent']
        
        # 2. LOGIC AI (Rule-based đơn giản)
        # Bạn có thể thay bằng Q-Learning ở đây nếu muốn phức tạp
        THRESHOLD_HIGH = 40.0 
        THRESHOLD_LOW = 20.0
        
        # Nếu CPU quá tải (>70%) và chưa giảm tải
        if sys_cpu > THRESHOLD_HIGH and self.current_load > 0.1:
            self.get_logger().warn(f'🔥 CPU Alert ({sys_cpu}%) > {THRESHOLD_HIGH}%! Throttling down...')
            self.current_load = 0.1
            self.set_load_level(self.current_load)
            
        
            
        # Nếu CPU rảnh (<30%) và đang bị kìm hãm
        elif sys_cpu < THRESHOLD_LOW and self.current_load < 1.0:
            self.get_logger().info(f'❄️ CPU Safe ({sys_cpu}%) < {THRESHOLD_LOW}%. Boosting up...')
            self.current_load = 1.0
            self.set_load_level(self.current_load)

def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()