import rclpy
from rclpy.node import Node

from math import pi

from canopen_motor_module.motor_management.motor_controller import MotorController
from canopen_motor_module.motor_management.motor_factory import MotorFactory

import json
import os
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState

class CANopenManagerNode(Node):
    def __init__(self):
        super().__init__('canopen_manager_node')
        self.get_logger().info('CANopen Manager Node has been started')

        self.controller = MotorController(channel='can0', bustype='socketcan', bitrate=1000000)
    
        self.json_open()
        self.motor_active_all()
        #self.controller.set_position(11, pi)
        #self.controller.set_position(2, pi)
        #self.controller.set_position(3, pi)
        #self.controller.set_position(4, pi)

        # JointState 구독자 생성
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',  # 토픽 이름
            self.joint_state_callback,
            10  # QoS 설정
        )

    def json_open(self):
        # json 파일 경로 설정
        package_path = get_package_share_directory('canopen_manager_pkg')
        json_path = os.path.join(package_path, 'json_motor_test', 'canopen_motor_elmo_test.json')

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
                profile_deceleration=motor.get('profile_deceleration', 0),
                name=motor.get('name', f"joint_{motor['node_id']}") #기본값 joint_node_id
            )
            motors.append(motor_obj)

        for motor in motors:
            self.controller.add_motor(motor)

    def motor_active_all(self):
        self.controller.reset_all()
        self.controller.init_all()
        self.controller.pdo_mapping_all()
        self.controller.set_switchOn_all()
        self.controller.pdo_callback_register_all()
        self.controller.sync_start(0.01)

    def joint_state_callback(self, msg):
        """JointState 메시지 콜백 함수"""
        try:
            # 배열 크기 확인
            if len(msg.name) != len(msg.position):
                self.get_logger().error('Invalid JointState message: arrays size mismatch')
                return

            # 각 조인트에 대해 위치 설정
            for joint_name, position in zip(msg.name, msg.position):
                try:
                    # 이름으로 직접 모터 위치 설정
                    self.controller.set_position_by_name(joint_name, position)
                except Exception as e:
                    self.get_logger().error(f'Failed to set position for joint {joint_name}: {str(e)}')

        except Exception as e:
            self.get_logger().error(f'Error processing JointState message: {str(e)}')

    def on_shutdown(self):
        """노드 종료 시 호출되는 정리 함수"""
        try:
            self.controller.sync_stop()  # sync 통신 중지
            self.controller.reset_all()  # 모든 모터 리셋
            self.get_logger().info('CANopen Manager Node has been safely shutdown')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = CANopenManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()  # 종료 전 정리 함수 호출
        node.destroy_node()
        # context가 아직 활성화 상태인 경우에만 shutdown 호출
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
