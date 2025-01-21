import rclpy
from rclpy.node import Node
from canopen_motor_module.motor_management.motor_controller import MotorController
from canopen_motor_module.motor_management.motor_factory import MotorFactory

import json
import os
from ament_index_python.packages import get_package_share_directory

TEST_ID = 11
class CANopenManagerNode(Node):
    def __init__(self):
        super().__init__('canopen_manager_node')
        self.get_logger().info('CANopen Manager Node has been started')

        controller = MotorController(channel='can0', bustype='socketcan', bitrate=1000000)



        # json 파일 경로 설정
        package_path = get_package_share_directory('canopen_manager_pkg')
        json_path = os.path.join(package_path, 'json_motor_test', 'canopen_motor_list_test.json')

        # json 파일 읽기
        with open(json_path, 'r') as f:
            motor_config = json.load(f)

        # json 파일의 모터 정보를 바탕으로 모터 객체 생성
        motors = []
        for motor in motor_config['motors']:
            motor_obj = MotorFactory.create_motor(
                vendor_type=motor['vendor_type'],
                node_id=motor['node_id'],
                zero_offset=motor['zero_offset'],
                operation_mode=motor['operation_mode'],
                profile_velocity=motor.get('profile_velocity', 0),
                profile_acceleration=motor.get('profile_acceleration', 0),
                profile_deceleration=motor.get('profile_deceleration', 0)
            )
            motors.append(motor_obj)

        for motor in motors:
            controller.add_motor(motor)

        controller.reset_all()
        controller.init_all()
        controller.pdo_mapping_all()
        controller.set_switchOn_all()
        controller.pdo_callback_register_all()
        controller.sync_start(0.01)
        controller.set_position(11, 262144)
        controller.set_position(2, 262144)
        controller.set_position(3, 262144)
        
        

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
