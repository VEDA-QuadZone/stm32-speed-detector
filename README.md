# STM32 기반 차량 과속 감지 시스템

## 개요

이 프로젝트는 **STM32F401RE** 보드를 기반으로 한 차량 과속 감지 시스템입니다. 두 쌍의 적외선 센서를 사용해 차량의 진입과 이탈 시점을 감지하고, 주행 속도를 계산하여 **과속 차량에 대한 정보를 Bluetooth 무선 통신을 통해 Raspberry Pi 등 외부 장치로 전송**합니다.

---

## 시스템 구성
<img width="1344" height="334" alt="과속 감지 시스템" src="https://github.com/user-attachments/assets/bbfdfef9-c2d6-4abd-a738-e4d3e4a3234d" />

### 하드웨어

| 구성 요소 | 설명 |
|-----------|------|
| STM32F401RE | 센서 신호 처리 및 속도 계산 MCU |
| 적외선 센서 4개 | 차량 감지를 위한 A1/A2, B1/B2 |
| HC-05 Bluetooth 모듈 | STM32 → Raspberry Pi 간 무선 통신 |
| Raspberry Pi | 과속 데이터 수신 및 처리 주체 |

---

## 센서 핀 구성

| 센서 이름 | STM32 핀 (Nucleo 기준) | 설명 |
|-----------|-------------------------|------|
| A1        | PA0                    | A차선 진입 센서 |
| A2        | PA1                    | A차선 이탈 센서 |
| B1        | PB5                    | B차선 진입 센서 |
| B2        | PB6                    | B차선 이탈 센서 |

- A1 → A2, B1 → B2 사이의 고정 거리: **0.5m**

---

## 프로토타입 모형
![도로 뒷판](https://github.com/user-attachments/assets/57cbb838-7f3c-4d4b-81d0-60c81ea2938f)
![20250801_183428(1)](https://github.com/user-attachments/assets/d0fe94dc-b63c-4ec2-995f-daacea75eceb)

---

## 속도 계산 및 과속 판별

- 측정 거리: `0.5m`
- 속도 계산 공식:

<pre> 속도(km/h) = (거리(m) / 측정 시간(s)) × 3.6 </pre>

- 과속 기준: **1.0 km/h 초과**
- 과속 시 메시지 전송

| 차선 | 메시지 포맷        | 설명              |
|------|-------------------|-------------------|
| A    | `3.45 IN\r\n`     | A차선 과속 차량   |
| B    | `4.10 OUT\r\n`    | B차선 과속 차량   |

---

## Bluetooth 연동

### 연결 구성

| HC-05 핀명 | STM32 핀 (Nucleo-F401RE 기준) | 설명 |
|------------|-------------------------------|------|
| **TXD**    | **PC7 (USART6_RX)**           | 5V → 3.3V 분압 필요 |
| **RXD**    | **PC6 (USART6_TX)**           | 직접 연결 가능 |
| **VCC**    | 5V 또는 3.3V                   | 모듈에 따라 다름 |
| **GND**    | GND                            | 공통 GND |

> 주의: HC-05 TXD는 5V 레벨 → STM32 RX 핀에는 **저항 분압 회로** 필요

### STM32CubeMX 설정 (USART6)

- Mode: `Asynchronous`
- Baud Rate: `9600`
- Word Length: `8 Bits`
- Parity: `None`
- Stop Bits: `1`
- Hardware Flow Control: `None`

### 데이터 흐름

```text
차량 감지
  ↓
속도 계산
  ↓
UART6 TX → HC-05
  ↓
Bluetooth 무선 전송
  ↓
Raspberry Pi 또는 PC에서 수신
````

### Raspberry Pi 수신 예시

```bash
sudo rfcomm bind /dev/rfcomm0 XX:XX:XX:XX:XX:XX
cat /dev/rfcomm0
```

```text
3.87 IN
4.12 OUT
```

---

## 소프트웨어 구조

### 주요 함수

| 함수                         | 설명                     |
| -------------------------- | ---------------------- |
| `HAL_GPIO_EXTI_Callback()` | 센서 인터럽트 진입점            |
| `process_sensor_event()`   | 진입/이탈 처리, 속도 계산, 과속 판별 |
| `check_queue_timeouts()`   | 대기 시간 초과한 차량 제거        |

### 차량 다중 감지 처리

* 각 차선당 **원형 큐 구조** 사용
* 최대 차량 수: `10대`
* 중복 감지 방지: 센서당 최소 3초 간격
* 비정상 측정 (10초 초과)은 무시

---

## UART 메시지 예시

```text
3.21 IN
4.95 OUT
```

* IN: A차선 과속 차량
* OUT: B차선 과속 차량
* 메시지는 HC-05를 통해 상위 장치로 전송

---

## 트러블슈팅

| 현상          | 원인                      | 해결 방법                       |
| ----------- | ----------------------- | --------------------------- |
| 메시지가 자주 전송됨 | 중복 감지 미차단               | `MIN_DETECTION_INTERVAL` 확인 |
| 차량 감지 실패    | 센서 설치 거리 이상             | 0.5m 거리 및 방향 확인             |
| UART 수신 실패  | TX/RX 핀 교차 오류 또는 전압 불일치 | 핀 배치 및 레벨 시프팅 회로 확인         |

