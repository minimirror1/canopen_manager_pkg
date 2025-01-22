import os
from ament_index_python.packages import get_package_share_directory
from ..motor_vendor.motorVendorZeroErr import MotorVendorZeroErr
from ..motor_vendor.motorVenderTemplate import MotorVendorB

# 필요하다면, 제조사 정보를 바탕으로 인스턴스를 생성해주는 Factory 구현 예시
class MotorFactory:
    @staticmethod
    def create_motor(vendor_type, node_id, zero_offset=0, operation_mode='PROFILE_POSITION',
                    profile_velocity=1.0, profile_acceleration=1.0, profile_deceleration=1.0,
                    name=None):
        """모터 객체 생성 팩토리 메서드
        :param vendor_type: 제조사 타입 (예: "VendorZeroErr")
        :param node_id: CAN 노드 ID
        :param eds_path: EDS 파일 경로
        :param zero_offset: 영점 오프셋
        :param operation_mode: 동작 모드 ('PROFILE_POSITION', 'PROFILE_TORQUE' 등)
        :param profile_velocity: 프로파일 속도 (rad/s)
        :param profile_acceleration: 프로파일 가속도 (rad/s²)
        :param profile_deceleration: 프로파일 감속도 (rad/s²)
        :param name: 조인트 이름
        """
        if vendor_type == "VendorZeroErr":
            # EDS 파일 경로 수정
            package_path = get_package_share_directory('canopen_manager_pkg')
            eds_path = os.path.join(package_path, 'config', 'ZeroErr Driver_V1.5.eds')
            return MotorVendorZeroErr(node_id, eds_path, zero_offset, operation_mode,
                                    profile_velocity, profile_acceleration, profile_deceleration,
                                    name)
        elif vendor_type == "VendorB":
            return MotorVendorB(node_id, eds_path, zero_offset, operation_mode)
        else:
            raise ValueError(f"Unknown vendor type: {vendor_type}")