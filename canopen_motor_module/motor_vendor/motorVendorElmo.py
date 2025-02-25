import canopen
from ..motor_management.abstract_motor import AbstractMotor
import logging
import time
from math import pi
import csv
from datetime import datetime

class MotorVendorElmo(AbstractMotor):
    """Elmo 모터에 대한 구체 구현."""
    PULSE_PER_REVOLUTION = 131072  # Elmo 모터의 한 바퀴당 펄스 수
    
    def __init__(self, node_id, eds_path, zero_offset=0, operation_mode='PROFILE_POSITION',
                 profile_velocity=1.0, profile_acceleration=1.0, profile_deceleration=1.0, name=None):
        super().__init__(node_id, eds_path, zero_offset, operation_mode,
                        profile_velocity, profile_acceleration, profile_deceleration,
                        name)
        self.current_velocity_old = 0
        self.dt = 0.001  # 1ms
        
    def init(self, operation_mode=None):

        self.network.nmt.send_command(0x02)  # Stop
        time.sleep(0.5)
        self.network.nmt.send_command(0x82)  # Reset
        time.sleep(1)  # 재설정 후 충분한 대기 시간
        self.network.nmt.send_command(0x01)  # Reset
        time.sleep(1)  # 재설정 후 충분한 대기 시간
        self.network.nmt.send_command(0x80)  # Reset
        time.sleep(1)  # 재설정 후 충분한 대기 시간

        self.node.sdo['controlword'].raw = 0x80  # Fault reset
        # a
        self.node.sdo['modes_of_operation'].raw = 0x01  # PROFILE_POSITION
        time.sleep(0.1)

        # b
        self.node.sdo['following_error_window'].raw = 3000
        time.sleep(0.1)
        self.node.sdo['max_profile_velocity'].raw = 160000
        time.sleep(0.1)
        self.node.sdo['profile_velocity'].raw = 160000
        time.sleep(0.1)
        self.node.sdo['profile_acceleration'].raw = 160000
        time.sleep(0.1)
        self.node.sdo['profile_deceleration'].raw = 160000
        time.sleep(0.1)
        self.node.sdo['quick_stop_deceleration'].raw = 160000
        time.sleep(0.1)
        self.node.sdo['motion_profile_type'].raw = 0
        time.sleep(0.1)

        # c
        self.node.sdo['controlword'].raw = 0x06  # Shutdown
        time.sleep(0.1)        
        self.node.sdo['controlword'].raw = 0x0F  # Operation enabled
        time.sleep(0.1)

        # d
        self.node.sdo['target_position'].raw = 500000
        time.sleep(0.1)

        # e
        self.node.sdo['controlword'].raw = 0x1F # Start
        time.sleep(0.1)

        # f
        self.node.sdo['controlword'].raw = 0x0F # Start
        time.sleep(0.1)

        for _ in range(50):  # 10초 동안 0.2초 간격으로 50회 반복
            position_actual_value = self.node.sdo['position_actual_value'].raw
            print(f"[MotorVendorElmo] Current actual value: {position_actual_value}")
            time.sleep(0.2)

        # if operation_mode:
        #     self.operation_mode = operation_mode.upper()
        
        # if self.operation_mode not in self.OPERATION_MODES:
        #     raise ValueError(f"지원하지 않는 동작 모드입니다: {self.operation_mode}")
            
        # print(f"[MotorVendorElmo] Init motor node: {self.node_id}")
        
        # # 모드 설정
        # mode_value = self.OPERATION_MODES[self.operation_mode]
        # self.node.sdo['modes_of_operation'].raw = mode_value
        
        # self.plusToRad = 2 * pi / self.PULSE_PER_REVOLUTION
        
        # # Disable sync
        # self.network.sync.stop()
        
        # 모드별 초기화
        #self._init_mode_specific_parameters()

    def pdo_mapping(self):
        print(f"[MotorVendorElmo] PDO mapping for node: {self.node_id}")
        # PDO 설정 읽기
        self.node.tpdo.read()
        self.node.rpdo.read()

        # TPDO 매핑 (모터 -> 컨트롤러)
        self.node.tpdo[1].clear()
        self.node.tpdo[1].add_variable('statusword')
        self.node.tpdo[1].add_variable('position_actual_value')
        self.node.tpdo[1].enabled = True

        self.node.tpdo[2].clear()
        self.node.tpdo[2].add_variable('torque_actual_value')
        self.node.tpdo[2].add_variable('velocity_actual_value')
        self.node.tpdo[2].enabled = True

        # RPDO 매핑 (컨트롤러 -> 모터)
        self.node.rpdo[1].clear()
        self.node.rpdo[1].add_variable('controlword')
        self.node.rpdo[1].add_variable('target_position')
        self.node.rpdo[1].enabled = True

        self.node.rpdo[2].clear()
        self.node.rpdo[2].add_variable('target_torque')
        self.node.rpdo[2].enabled = True

        # PDO 설정 저장 및 적용
        self.node.nmt.state = 'PRE-OPERATIONAL'
        self.node.tpdo.save()
        self.node.rpdo.save()
        self.node.nmt.state = 'OPERATIONAL'

    def set_switchOn(self):
        print(f"[MotorVendorElmo] Set switch on, node: {self.node_id}")
        # Elmo 모터의 상태 전환 시퀀스
        # self.node.rpdo[1]['controlword'].raw = 0x06  # Shutdown
        # time.sleep(0.1)
        # self.node.rpdo[1]['controlword'].raw = 0x07  # Switch on
        # time.sleep(0.1)
        # self.node.rpdo[1]['controlword'].raw = 0x0F  # Operation enabled
        # time.sleep(0.1)
        self.node.sdo['controlword'].raw = 0x06  # Shutdown
        time.sleep(0.1)
        self.node.sdo['controlword'].raw = 0x07  # Switch on
        time.sleep(0.1)
        self.node.sdo['controlword'].raw = 0x0F  # Operation enabled
        time.sleep(0.1)

    def _convert_rad_to_pulse(self, rad_value):
        """라디안 값을 펄스 카운트로 변환"""
        return int((rad_value * self.PULSE_PER_REVOLUTION) / (2 * pi))

    def set_position(self, value):
        position_pulse = self._convert_rad_to_pulse(value) + self.zero_offset
        
        #self.node.rpdo[1]['target_position'].raw = position_pulse
        #self.node.rpdo[1].transmit()
        self.node.sdo['target_position'].raw = position_pulse
        

    def get_position(self):
        return self.current_position

    def set_torque(self, value):
        self.node.rpdo[2]['target_torque'].raw = value
        self.node.rpdo[2].transmit()

    def get_torque(self):
        return self.current_torque

    def pdo_callback_register(self):
        self.network.subscribe(self.node.tpdo[1].cob_id, self.node.tpdo[1].on_message)
        self.node.tpdo[1].add_callback(self.tpdo1_callback)
        
        self.network.subscribe(self.node.tpdo[2].cob_id, self.node.tpdo[2].on_message)
        self.node.tpdo[2].add_callback(self.tpdo2_callback)

    def tpdo1_callback(self, message):
        position = int.from_bytes(message.data[2:6], byteorder='little', signed=True)
        self.current_position = (position - self.zero_offset) * self.plusToRad

    def tpdo2_callback(self, message):
        self.current_torque = int.from_bytes(message.data[0:2], byteorder='little', signed=True)
        velocity_pulse = int.from_bytes(message.data[2:6], byteorder='little', signed=True)
        self.current_velocity = velocity_pulse * self.plusToRad
        
        self.current_acceleration = (self.current_velocity - self.current_velocity_old) / self.dt
        self.current_velocity_old = self.current_velocity

    def reset(self):
        print(f"[MotorVendorElmo] Reset motor node: {self.node_id}")
        # self.node.sdo[0x6040].raw = 0x80  # Fault reset
        # time.sleep(0.1)
        # self.node.sdo[0x6040].raw = 0x06  # Shutdown
        # time.sleep(0.1)
        # self.node.sdo[0x6040].raw = 0x07  # Switch on
        # time.sleep(0.1)
        # self.node.sdo[0x6040].raw = 0x0F  # Operation enabled
        # time.sleep(0.1)
        self.node.sdo['controlword'].raw = 0x80  # Fault reset
        time.sleep(0.1)
        self.node.sdo['controlword'].raw = 0x06  # Shutdown
        time.sleep(0.1)
        self.node.sdo['controlword'].raw = 0x07  # Switch on
        time.sleep(0.1)
        self.node.sdo['controlword'].raw = 0x0F  # Operation enabled

    def get_velocity(self):
        return self.current_velocity

    def get_acceleration(self):
        return self.current_acceleration

    def set_velocity(self, value):
        print(f"[MotorVendorElmo] Set velocity to {value}, node: {self.node_id}")
        velocity_pulse = self._convert_rad_to_pulse(value)
        self.node.sdo['profile_velocity'].raw = velocity_pulse

    def set_acceleration(self, value):
        print(f"[MotorVendorElmo] Set acceleration to {value}, node: {self.node_id}")
        acceleration_pulse = self._convert_rad_to_pulse(value)
        self.node.sdo['profile_acceleration'].raw = acceleration_pulse

    def log_start(self):
        """로그 시작"""
        self.logging = True
        self.start_time = time.time()
        
        # 현재 시간을 이용한 파일명 생성
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"motor_log_{self.node_id}_{timestamp}.csv"
        
        # CSV 파일 생성 및 헤더 작성
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['Time(ms)', 'Position(rad)', 'Torque(Nm)', 'Velocity(rad/s)', 'Acceleration(rad/s^2)'])

    def log_stop(self):
        """로그 종료"""
        if hasattr(self, 'logging') and self.logging:
            self.logging = False
            self.log_file.close()
 