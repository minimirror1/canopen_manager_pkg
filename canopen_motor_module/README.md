# 모터 제어 시스템

## 개요
이 프로젝트는 CAN 통신을 이용한 다중 모터 제어 시스템입니다. 사인파 형태의 위치 제어를 통해 여러 모터를 동시에 제어할 수 있습니다.

## 주요 기능
- 다중 모터 동시 제어
- 실시간 사인파 위치 제어
- PDO 매핑을 통한 효율적인 통신
- 모터별 제로 포인트 오프셋 설정
- 모듈화된 모터 드라이버 구조

## 시스템 구조

### 핵심 클래스
- `AbstractMotor`: 모터 인터페이스 정의
- `MotorController`: 다중 모터 제어 관리
- `MotorFactory`: 모터 객체 생성 관리
- `MotorVendorZeroErr`: ZeroErr 드라이버 구현

### 주요 기능 구현
- 모터 초기화 및 설정
- PDO 매핑 및 콜백 등록
- 실시간 위치 제어
- 에러 처리 및 복구

## 설치 요구사항
- Python 3.10 이상
- python-can
- canopen

## 사용 방법

### 1. 모터 초기화
```python
# PCAN 의 경우
controller = MotorController(channel='can0', bustype='socketcan', bitrate=1000000)
# USB-CAN 의 경우
controller = MotorController(interface='slcan', channel='COM3', bitrate=1000000)

motorA = MotorFactory.create_motor("VendorZeroErr", 1, "config/ZeroErr Driver_V1.5.eds", zero_offset=84303)
controller.add_motor(motorA)
```

### 2. 기본 설정
```python
controller.reset_all()
controller.init_all()
controller.pdo_mapping_all()
controller.set_switchOn_all()
controller.pdo_callback_register_all()
```


### 3. 위치 제어 실행
```python
controller.sync_start(0.01) # 10ms 주기
controller.set_position_all(target_position)
```

### 4. 종료
```python
controller.sync_stop()
controller.shutdown_all()
```

## 주의사항
- CAN 버스 설정이 올바르게 되어있는지 확인
- 모터 ID가 중복되지 않도록 주의
- 제로 오프셋 값 설정 시 주의 필요

## 라이선스
이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 기여 방법
1. 이슈 등록
2. Pull Request 제출
3. 코드 리뷰 진행
4. 승인 및 머지

## 문의사항
기술적인 문의사항이나 버그 리포트는 이슈 트래커를 이용해 주시기 바랍니다.