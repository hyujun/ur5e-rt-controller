# ur5e-rt-controller 워크스페이스 분석

**경로**: `/home/junho/ros2_ws/ur5e_ws/src/ur5e-rt-controller`  
**환경**: Ubuntu 22.04 (ROS2 Humble) / Ubuntu 24.04 (ROS2 Jazzy), C++20, `ament_cmake`

---

## 전체 구조 개요

```
ur5e-rt-controller/          ← 레포지토리 루트 (v5.0.0+: src/ 없음)
├── install.sh               ← 자동 설치 (의존성 + 빌드 + RT 권한)
├── build.sh
├── docs/                    ← CHANGELOG, RT_OPTIMIZATION, CORE_ALLOCATION
│
├── ur5e_rt_base/            ← [헤더-전용] 공유 기반 패키지
├── ur5e_rt_controller/      ← [핵심] 500Hz RT 제어기 패키지
├── ur5e_hand_udp/           ← UDP ↔ ROS2 핸드 브리지
├── ur5e_mujoco_sim/         ← MuJoCo 3.x 시뮬레이터 (선택적)
└── ur5e_tools/              ← Python 개발 유틸리티
```

---

## 패키지별 역할

### 1. `ur5e_rt_base` — 공유 기반 (헤더-전용)

구현 파일 없음. `INTERFACE` CMake 라이브러리.

| 헤더 | 내용 |
|------|------|
| `types.hpp` | `RobotState`, `HandState`, `ControllerState`, `ControllerOutput` 구조체 + 상수 |
| `thread_config.hpp` | `ThreadConfig` + 4/6/8코어 RT 스레드 배치 상수 |
| `thread_utils.hpp` | `ApplyThreadConfig()`, `SelectThreadConfigs()` |
| `log_buffer.hpp` | `SpscLogBuffer<512>` — lock-free SPSC 링 버퍼 |
| `data_logger.hpp` | `DataLogger` — CSV 파일 기록 (log 스레드 전용) |
| `filters/bessel_filter.hpp` | `BesselFilterN<N>` — 4차 Bessel LPF (`noexcept`) |
| `filters/kalman_filter.hpp` | `KalmanFilterN<N>` — 이산-시간 Kalman 필터 (`noexcept`) |

**핵심 타입:**
```cpp
namespace ur5e_rt_controller {  // alias: urtc
  inline constexpr int kNumRobotJoints = 6;
  inline constexpr int kNumHandJoints  = 11;
  inline constexpr int kNumHandSensors = 44;  // 4 × 11

  struct RobotState   { array<double,6> positions, velocities; array<double,3> tcp_position; double dt; uint64_t iteration; };
  struct HandState    { array<double,11> motor_positions, velocities, currents; array<double,44> sensor_data; bool valid; };
  struct ControllerState   { RobotState robot; HandState hand; double dt; uint64_t iteration; };
  struct ControllerOutput  { array<double,6> robot_commands; array<double,11> hand_commands; bool valid; };
}
```

---

### 2. `ur5e_rt_controller` — 500Hz 실시간 제어기 (핵심)

**유일한 실행 파일**: `src/custom_controller.cpp`

#### 스레딩 모델 — 4 SingleThreadedExecutor + 4 std::thread

| 스레드 | 코어 | 스케줄러 | 우선순위 | 역할 |
|--------|------|---------|---------|------|
| `t_rt` | Core 2 | SCHED_FIFO | 90 | `ControlLoop()` 500Hz, `CheckTimeouts()` 50Hz |
| `t_sensor` | Core 3 | SCHED_FIFO | 70 | `/joint_states`, `/target_joint_positions`, `/hand/joint_states` 구독 |
| `t_log` | Core 4 | SCHED_OTHER | nice -5 | CSV 드레인 (100Hz) |
| `t_aux` | Core 5 | SCHED_OTHER | 0 | E-STOP 상태 퍼블리시 |

#### 코어 할당 (코어 수에 따른 자동 선택)

| 코어 수 | RT | Sensor | UDP recv | Log | Aux |
|---------|-----|--------|---------|-----|-----|
| 6코어 | 2 | 3 | 5 | 4 | 5 |
| 8코어 | 2 | 3 | 4 | 5 | 6 |
| 4코어(fallback) | 1 | 2 | 2 | 3 | 3 |

#### 뮤텍스 구조

| 데이터 | 보호 뮤텍스 | 접근 |
|--------|------------|------|
| `current_pos_/vel_` | `state_mutex_` | sensor(W), RT(R) |
| `target_snapshot_` | `target_mutex_` | sensor(W), RT(R) |
| 핸드 타임스탬프 | `hand_mutex_` | sensor(W), RT(R) |
| E-STOP 플래그 | `atomic<bool>` | RT(R), timeout(W) |

#### 제어기 (Strategy 패턴)

| 제어기 | 제어 공간 | 비고 |
|--------|----------|------|
| `PDController` | 관절-공간 PD | **기본 활성**. E-STOP 시 `kSafePosition`으로 이동 |
| `PController` | 관절-공간 P | E-STOP 없음 |
| `PinocchioController` | 관절-공간 PD + 동역학 | 중력 + Coriolis 보상 |
| `ClikController` | Cartesian 3-DOF | 감쇠 Jacobian 유사역행렬 + null-space |
| `OperationalSpaceController` | Cartesian 6-DOF | SO(3) 오리엔테이션 오차 (`log3()`) |

**`/target_joint_positions` 해석:**
- `PDController` / `Pinocchio`: 6개 관절 각도 (rad)
- `ClikController`: `[x, y, z, null_q3, null_q4, null_q5]`
- `OSC`: `[x, y, z, roll, pitch, yaw]`

#### RT 경로 규칙 (ControlLoop() 절대 금지)
- ❌ `std::mutex::lock()` → `try_lock()` 사용
- ❌ `RCLCPP_INFO` / `std::cout` → `log_buffer_.Push()` 경유
- ❌ `new` / `delete` / STL 컨테이너 resize
- ❌ 파일 I/O

#### E-STOP 시스템

`CheckTimeouts()` (50Hz):
- `/joint_states` 공백 > `robot_timeout_ms(100ms)` → `PDController::TriggerEstop()`
- `/hand/joint_states` 공백 > `hand_timeout_ms(200ms)` → `SetHandEstop(true)`
- E-STOP 시 타겟 → `kSafePosition = [0, -1.57, 1.57, -1.57, -1.57, 0]` rad, 최대 `2.0 rad/s`

---

### 3. `ur5e_hand_udp` — UDP 핸드 브리지

11-DOF 커스텀 핸드 ↔ ROS2 UDP 양방향 브리지.

#### UDP 패킷 프로토콜

**수신 (포트 50001)**: 77 double = 616 bytes
| 필드 | 크기 |
|------|------|
| `motor_positions[11]` | 88B |
| `motor_velocities[11]` | 88B |
| `motor_currents[11]` | 88B |
| `sensor_data[44]` (4센서 × 11관절) | 352B |

**송신 (포트 50002)**: 11 double = 88 bytes (정규화 모터 명령 0.0–1.0)

#### 노드

- **`hand_udp_receiver_node`**: UDP 수신 → `/hand/joint_states` (100Hz) 발행. `jthread` (Core 5, FIFO 65)
- **`hand_udp_sender_node`**: `/hand/command` 구독 → UDP 송신

> **주의**: `ur5e_mujoco_sim`과 동시 실행 시 `/hand/joint_states` 토픽 충돌.

---

### 4. `ur5e_mujoco_sim` — MuJoCo 시뮬레이터

**선택적 의존성** — MuJoCo 미설치 시 CMake가 패키지 전체 스킵.

#### 스레딩 모델

```
mujoco_simulator_node (ROS2 spin)
    ├── SimLoop (jthread)       ← mj_step() + InvokeStateCallback()
    └── ViewerLoop (jthread)    ← GLFW ~60Hz 렌더링 (MUJOCO_HAVE_GLFW)
```

#### 동기화 원칙

| 뮤텍스 | 보호 대상 | 규칙 |
|--------|----------|------|
| `cmd_mutex_` | `pending_cmd_` | SetCommand() ↔ ApplyCommand() |
| `cmd_pending_` (atomic) | FreeRun fast path | lock-free 커맨드 확인 |
| `sync_cv_` | SyncStep 대기 | 커맨드 도착 시 notify |
| `state_mutex_` | `latest_positions_/velocities_/efforts_` | ReadState(W), GetPositions(R) |
| `viz_mutex_` | `viz_qpos_` | **try_lock 전용** — SimLoop 절대 대기 안함 |

#### 시뮬레이션 모드

| 모드 | `SimMode` | 설명 |
|------|-----------|------|
| Free-run | `kFreeRun` | `mj_step()` 최대 속도. 알고리즘 검증용 |
| Sync-step | `kSyncStep` | 퍼블리시 → 커맨드 대기 → step. 지연 측정용 |

#### 발행 토픽
- `/joint_states` (positions + velocities + efforts)
- `/hand/joint_states` (100Hz, 1차 필터 시뮬)
- `/sim/status` (`[step_count, sim_time_sec, rtf, paused]`)

---

### 5. `ur5e_tools` — Python 유틸리티 (`ament_python`)

C++ 코드 없음. `ros2 run ur5e_tools <script>` 실행.

| 스크립트 | 의존성 | 역할 |
|---------|--------|------|
| `monitor_data_health.py` | rclpy | 4개 토픽 상태 감시 (10Hz), JSON 통계 저장 |
| `plot_ur_trajectory.py` | pandas, matplotlib | CSV 로그 시각화 (ROS2 불필요) |
| `motion_editor_gui.py` | PyQt5 | 50-포즈 모션 에디터 GUI |
| `hand_udp_sender_example.py` | — | 가상 핸드 UDP 데이터 생성기 (500Hz) |

---

## ROS2 토픽 전체 맵

| 토픽 | 타입 | 발행자 | 구독자 |
|------|------|--------|--------|
| `/joint_states` | `sensor_msgs/JointState` | UR driver / MuJoCo sim | `custom_controller` |
| `/target_joint_positions` | `std_msgs/Float64MultiArray` | 사용자 / GUI | `custom_controller` |
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | UDP receiver / MuJoCo sim | `custom_controller` |
| `/hand/command` | `std_msgs/Float64MultiArray` | `custom_controller` | UDP sender |
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | `custom_controller` | UR driver / MuJoCo sim |
| `/system/estop_status` | `std_msgs/Bool` | `custom_controller` | — |
| `/sim/status` | `std_msgs/Float64MultiArray` | MuJoCo sim | — |

---

## 성능 특성

| 지표 | v4.2.0+ |
|------|---------|
| 제어 지터 | < 50μs |
| E-STOP 반응 | < 20ms |
| CPU 사용률 | ~25% |
| 컨텍스트 스위치 | ~1000/s |

**`mlockall(MCL_CURRENT | MCL_FUTURE)`**: `rclcpp::init()` 이전에 호출 (페이지 폴트 방지).

---

## 빌드 & 실행

```bash
# 자동 설치
chmod +x install.sh && ./install.sh

# 수동 빌드
cd ~/ur_ws
colcon build --packages-select ur5e_rt_base ur5e_rt_controller --symlink-install
source install/setup.bash

# 실제 로봇
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=192.168.1.10

# MuJoCo 시뮬
ros2 launch ur5e_rt_controller mujoco_sim.launch.py [sim_mode:=sync_step]

# Fake hardware (로봇 불필요)
ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true
```

---

## 커스텀 제어기 추가 방법

```cpp
// 1. include/ur5e_rt_controller/controllers/my_ctrl.hpp 생성
class MyController final : public RTControllerInterface {
 public:
  [[nodiscard]] ControllerOutput Compute(const ControllerState& s) noexcept override;
  void SetRobotTarget(std::span<const double, 6> t) noexcept override;
  void SetHandTarget(std::span<const double, 11> t) noexcept override;
  [[nodiscard]] std::string_view Name() const noexcept override { return "MyController"; }
};

// 2. custom_controller.cpp 생성자에서 controller_ 초기화만 교체
// CMakeLists 변경 불필요 (헤더-전용)
```

> **필수**: 모든 가상 메서드는 `noexcept`. Eigen 버퍼는 생성자에서 사전 할당.

---

## 코드 컨벤션

- **네임스페이스**: `ur5e_rt_controller` (`.cpp`에서 `namespace urtc`로 별칭)
- **C++20 기능**: `std::jthread`, `std::stop_token`, designated initializers, `std::concepts`, `std::span`
- **Google C++ Style**: `snake_case` 멤버 + trailing `_`, `[[nodiscard]]` 광범위 사용
- **경고**: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion` 경고 없이 컴파일 필수
- **Pinocchio 헤더**: `#pragma GCC diagnostic push/pop` 필수
- **Include 순서**: 프로젝트 헤더 → ROS2/서드파티 → C++ stdlib
