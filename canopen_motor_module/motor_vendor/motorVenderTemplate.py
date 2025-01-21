from ..motor_management.abstract_motor import AbstractMotor

class MotorVendorB(AbstractMotor):
    """제조사 B 모터에 대한 구체 구현."""
    def __init__(self, node_id, eds_path, zero_offset=0, operation_mode='PROFILE_POSITION'):
        super().__init__(node_id, eds_path, zero_offset, operation_mode)

    def init(self):
        print(f"[VendorB] Init motor node: {self.node_id}")
        pass

    def reset(self):
        print(f"[VendorB] Reset motor node: {self.node_id}")
        pass

    def pdo_mapping(self):
        print(f"[VendorB] PDO mapping for node: {self.node_id}")
        pass

    def set_position(self, value):
        print(f"[VendorB] Set position to {value}, node: {self.node_id}")
        pass

    def get_position(self):
        print(f"[VendorB] Get position, node: {self.node_id}")
        return 0
