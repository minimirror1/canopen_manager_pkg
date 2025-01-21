import rclpy
from rclpy.node import Node
from canopen_motor_module.motor_management.motor_controller import MotorController
from canopen_motor_module.motor_management.motor_factory import MotorFactory
TEST_ID = 11
class CANopenManagerNode(Node):
    def __init__(self):
        super().__init__('canopen_manager_node')
        self.get_logger().info('CANopen Manager Node has been started')

        controller = MotorController(channel='can0', bustype='socketcan', bitrate=1000000)
        motorA = MotorFactory.create_motor("VendorZeroErr", TEST_ID, "config/ZeroErr Driver_V1.5.eds", zero_offset=84303, operation_mode='PROFILE_POSITION')
        controller.add_motor(motorA)
        controller.reset_all()
        controller.init_all()
        controller.pdo_mapping_all()
        controller.set_switchOn_all()
        controller.pdo_callback_register_all()
        controller.sync_start(0.01)        
        
        

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
