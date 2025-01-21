import canopen
import time
import math
from motor_management.abstract_motor import AbstractMotor

from motor_management.motor_factory import MotorFactory
from motor_management.motor_controller import MotorController


import random

# 1 84,303
# 2 78,500
# 3 69,238
# 4 81,038

def get_wave_data(current_time):
    # frequency: Hz (주파수)
    # amplitude: 진폭
    # offset: 오프셋
    frequency = 0.1  # 60Hz
    amplitude = 50000
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

    # 예시: 모터 생성(제조사별)
    motorA = MotorFactory.create_motor("VendorZeroErr", 1, "config/ZeroErr Driver_V1.5.eds", zero_offset=84303)
    motorB = MotorFactory.create_motor("VendorZeroErr", 2, "config/ZeroErr Driver_V1.5.eds", zero_offset=78500)
    motorC = MotorFactory.create_motor("VendorZeroErr", 3, "config/ZeroErr Driver_V1.5.eds", zero_offset=69238)
    motorD = MotorFactory.create_motor("VendorZeroErr", 4, "config/ZeroErr Driver_V1.5.eds", zero_offset=81038)

    """motorA = MotorFactory.create_motor("VendorZeroErr", 1, "config/ZeroErr Driver_V1.5.eds", zero_offset=0)
    motorB = MotorFactory.create_motor("VendorZeroErr", 2, "config/ZeroErr Driver_V1.5.eds", zero_offset=0)
    motorC = MotorFactory.create_motor("VendorZeroErr", 3, "config/ZeroErr Driver_V1.5.eds", zero_offset=0)
    motorD = MotorFactory.create_motor("VendorZeroErr", 4, "config/ZeroErr Driver_V1.5.eds", zero_offset=0)"""
    #motorB = MotorFactory.create_motor("VendorB", 2, "vendorB.eds")

    # 컨트롤러에 모터 등록
    controller.add_motor(motorA)
    controller.add_motor(motorB)
    controller.add_motor(motorC)
    controller.add_motor(motorD)
    #time.sleep(3) 
    
    # 리셋
    controller.reset_all()
    #time.sleep(3) 
    
    # 모터 전체 초기화
    controller.init_all()
    #time.sleep(3) 
    
    # PDO 매핑
    controller.pdo_mapping_all()

    # Switch On
    controller.set_switchOn_all()

    # PDO 콜백 등록
    controller.pdo_callback_register_all()
   
    # 동기화 시작
    controller.sync_start(0.01)
    
    # 위치 설정
    
    controller.set_position_all(0)

    # time.sleep(5)

    current_time = 0
    try:
        while True:
            current_time = current_time + 0.01
            current_value = get_wave_data(current_time)
            # 모든 모터에 현재 파형 값을 위치로 설정
            controller.set_position_all(int(current_value))
            print(f"Current position: {current_value}, time: {current_time}")
            time.sleep(0.01)  # time_step(0.01초) 만큼 대기

    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다...")
    
    """
    # 위치 읽기
    for i in range(30):
        positions = controller.get_positions()
        print("Current positions:", positions)
        time.sleep(0.2) 

    controller.set_position_all(0)
    """

    """
    # 위치 읽기
    for i in range(30):
        positions = controller.get_positions()
        print("Current positions:", positions)
        time.sleep(0.2) 
    
    # 랜덤 위치 설정
    try:
        while True:
            # 랜덤 목표 위치 생성
            target_position = random.randint(0, 524288)
            print(f"\n새로운 목표 위치: {target_position}")
            
            # 목표 위치로 이동 명령
            controller.set_position_all(target_position)
            
            # 목표 위치 도달할 때까지 현재 위치 모니터링
            while True:
                positions = controller.get_positions()
                print("현재 위치들:", positions)
                
                # 모든 모터가 목표 위치에 도달했는지 확인
                all_reached = all(is_position_reached(pos, target_position) 
                                for pos in positions.values())
                
                if all_reached:
                    print("목표 위치에 도달했습니다!")
                    break
                    
                time.sleep(0.2)
                
    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다...")
    """

    """
    # 랜덤 모터 위치 제어
    try:
        motor_sel_cnt = 1  # 선택할 모터 개수 초기화
        
        while True:
            # 모터 선택 개수 업데이트
            selected_motors = random.sample(range(1, 5), motor_sel_cnt)
            # 랜덤 목표 위치 생성
            target_position = random.randint(0, 524288)
            
            print(f"\n선택된 모터 개수: {motor_sel_cnt}")
            print(f"선택된 모터: {selected_motors}, 목표 위치: {target_position}")
            
            # 선택된 모터들 이동
            for motor_id in selected_motors:
                controller.motors[motor_id].set_position(target_position)
            
            # 두든 선택된 모터가 목표 위치에 도달할 때까지 모니터링
            while True:
                all_reached = True
                for motor_id in selected_motors:
                    current_position = controller.motors[motor_id].get_position()
                    print(f"모터 {motor_id} 현재 위치: {current_position}")
                    if not is_position_reached(current_position, target_position):
                        all_reached = False
                
                if all_reached:
                    print(f"모터 {selected_motors}가 목표 위치에 도달했습니다!")
                    break
                    
                time.sleep(0.2)
            
            # 다음 루프를 위해 모터 선택 개수 증가
            motor_sel_cnt += 1
            if motor_sel_cnt > 4:
                motor_sel_cnt = 1
                
    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다...")

    """
    
    # 종료 전 네트워크 해제
    controller.disconnect()

if __name__ == "__main__":
    main()


    
