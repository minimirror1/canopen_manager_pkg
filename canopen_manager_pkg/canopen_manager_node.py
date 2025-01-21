import rclpy
from rclpy.node import Node

class CANopenManagerNode(Node):
    def __init__(self):
        super().__init__('canopen_manager_node')
        self.get_logger().info('CANopen Manager Node has been started')

def main(args=None):
    rclpy.init(args=args)
    
    node = CANopenManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
