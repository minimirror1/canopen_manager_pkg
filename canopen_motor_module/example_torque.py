import canopen
import serial
import time
import math
from motor_management.abstract_motor import AbstractMotor

from motor_management.motor_factory import MotorFactory
from motor_management.motor_controller import MotorController


import random

TEST_ID = 11

# 1 84,303
# 2 78,500
# 3 69,238
# 4 81,038

def get_wave_data(current_time):
    # frequency: Hz (주파수)s
    # amplitude: 진폭
    # offset: 오프셋
    frequency = 1   # 60Hz
    amplitude = 65536
    offset = 0
    
    # 실시간으로 사인파 계산
    # 2π * frequency * time => 각속도 * 시간
    current_value = amplitude * math.sin(2 * math.pi * frequency * current_time) + offset
    return int(current_value)
    
    
    
def is_position_reached(current_pos, target_pos, tolerance=10):
    return abs(current_pos - target_pos) <= tolerance

def main():
    # 예시: CAN Bus controller 생성
    controller = MotorController(channel='can0', bustype='socketcan', bitrate=1000000)
    #controller = MotorController(interface='slcan', channel='COM3', bitrate=1000000)
    #controller = MotorController(interface='slcan', channel='/dev/ttyACM0', bitrate=1000000)
    

    # 예시: 모터 생성(제조사별)
    #motorA = MotorFactory.create_motor("VendorZeroErr", TEST_ID, "config/ZeroErr Driver_V1.5.eds", zero_offset=84303, operation_mode='PROFILE_TORQUE')
    motorA = MotorFactory.create_motor("VendorZeroErr", TEST_ID, "config/ZeroErr Driver_V1.5.eds", zero_offset=84303, operation_mode='PROFILE_POSITION')
    
    # 컨트롤러에 모터 등록
    controller.add_motor(motorA)
    
    # 리셋
    controller.reset_all()    
    
    # 모터 전체 초기화
    controller.init_all()    
    
    # PDO 매핑
    controller.pdo_mapping_all()

    # Switch On
    controller.set_switchOn_all()

    # PDO 콜백 등록
    controller.pdo_callback_register_all()
   
    # 동기화 시작
    controller.sync_start(0.01)


    controller.set_position(TEST_ID, 0)

    time.sleep(2)
    # 로그 기록 시작
    controller.log_start(TEST_ID)
    
    # 토크 설정
    #controller.set_torque(TEST_ID, 200)

    cnt = 0
    try:
        while True:
            time.sleep(0.01)

            current_time = cnt * 0.01
            current_value = get_wave_data(current_time)
            #controller.set_position(TEST_ID, int(current_value))
            
            if cnt % 100 == 0:
                print(f"Torque: {controller.get_torque(TEST_ID)}")
                print(f"Velocity: {controller.get_velocity(TEST_ID)}")
                print(f"Acceleration: {controller.get_acceleration(TEST_ID)}")
            cnt += 1
    except KeyboardInterrupt:
        #controller.set_torque(TEST_ID, 0)
        # 로그 기록 종료
        controller.log_stop(TEST_ID)
        print("\n프로그램을 종료합니다...")
    
    # 종료 전 네트워크 해제
    controller.disconnect()

if __name__ == "__main__":
    main()


    
