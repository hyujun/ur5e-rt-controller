# MuJoCo 시뮬레이터 통합 계획

**작성일:** 2026-03-03
**버전:** 1.0.0
**목표:** 기존 ROS2 UR5e RT 제어기를 물리 로봇 없이 MuJoCo 시뮬레이터로 검증할 수 있는 환경 구축

---

## 1. 개요 및 목표

### 1.1 동기

현재 `custom_controller.cpp`는 실제 UR5e 로봇과 통신하는 구조로, 물리 로봇 없이 제어 알고리즘을 개발·검증하기 어렵다. MuJoCo 시뮬레이터를 ROS2 노드로 래핑하여 기존 제어기 코드를 **변경 없이** 시뮬레이션 환경에서 실행할 수 있게 한다.

### 1.2 핵심 설계 원칙

- **투명한 ROS2 인터페이스**: 시뮬레이터 노드는 실제 UR 드라이버와 동일한 토픽을 게시/구독하여 제어기 코드 수정 불필요
- **기존 아키텍처 존중**: 현재 4-스레드 멀티 executor 구조, PD/OSC/CLIK 제어기를 그대로 활용
- **단계적 구현**: 기본 관절 제어 → 시각화 → 핸드 시뮬레이션 순으로 확장

### 1.3 ROS2 토픽 매핑

```
[custom_controller 노드]              [mujoco_simulator 노드]
  Subscribe: /joint_states     ←←←  Publish:  /joint_states
  Publish: /forward_position_      →→→  Subscribe: /forward_position_
           controller/commands               controller/commands
  Subscribe: /hand/joint_states ←←← Publish:  /hand/joint_states (Phase 4)
```

---

## 2. 시스템 아키텍처

### 2.1 전체 구성도

```
┌─────────────────────────────────────────────────────────────┐
│                     ROS2 DDS 레이어                          │
│                                                             │
│  /joint_states ──────────────────────────────────────────► │
│  /forward_position_controller/commands ◄──────────────────── │
│  /target_joint_positions ────────────────────────────────► │
│  /system/estop_status ◄───────────────────────────────────  │
└─────────────────────────────────────────────────────────────┘
         ▲                                    ▲
         │                                    │
┌────────┴────────┐                ┌──────────┴──────────┐
│  MuJoCo         │                │  custom_controller  │
│  Simulator Node │                │  (기존 노드,         │
│                 │                │   변경 없음)          │
│  - UR5e MJCF 로드│               │                     │
│  - 물리 시뮬      │               │  - PDController      │
│  - 상태 게시      │               │  - OSC/CLIK 제어기    │
│  - 명령 수신      │               │  - E-STOP 시스템     │
│  - 뷰어 스레드    │               │  - 데이터 로깅        │
└─────────────────┘                └─────────────────────┘
```

### 2.2 MuJoCo 시뮬레이터 노드 스레드 구조

```
메인 스레드 (SCHED_OTHER)
  └─ rclcpp::init, 노드 생성, executor spin

시뮬레이션 스레드 (SCHED_FIFO, 우선순위 85, Core 0)
  └─ mj_step() @ 500Hz (또는 1kHz 내부 → 500Hz 게시)
  └─ /joint_states 게시
  └─ /hand/joint_states 게시 (Phase 4)

수신 스레드 (SCHED_FIFO, 우선순위 70, Core 1)
  └─ /forward_position_controller/commands 구독
  └─ ctrl 버퍼 업데이트

뷰어 스레드 (SCHED_OTHER, Core 6)
  └─ MuJoCo passive viewer @ 60Hz
  └─ 비활성화 가능 (headless 모드)
```

---

## 3. 구현 파일 목록

### 3.1 새로 생성할 파일

```
ur5e_rt_controller/
├── models/
│   └── ur5e/
│       ├── ur5e.xml              # UR5e MuJoCo MJCF 모델 (MuJoCo Menagerie)
│       ├── assets/               # 메시 파일 (.obj, .stl)
│       └── scene.xml             # 월드 + 로봇 씬 조합
│
├── include/ur5e_rt_controller/
│   └── mujoco_simulator.hpp      # MuJoCo 래퍼 클래스
│
├── src/
│   └── mujoco_simulator_node.cpp # ROS2 노드 진입점
│
├── config/
│   └── mujoco_simulator.yaml     # 시뮬레이터 파라미터
│
└── launch/
    └── mujoco_sim.launch.py      # 시뮬레이션 전용 런치 파일
```

### 3.2 수정할 기존 파일

| 파일 | 수정 내용 |
|------|-----------|
| `CMakeLists.txt` | MuJoCo 의존성 추가, `mujoco_simulator_node` 실행파일 추가 |
| `package.xml` | `<depend>` 항목에 MuJoCo 관련 패키지 추가 |

---

## 4. 단계별 구현 계획

---

### Phase 1: 환경 구성 및 UR5e MJCF 모델 준비

#### 1.1 MuJoCo 설치

```bash
# Python 바인딩 설치 (개발·검증용)
pip install mujoco

# C++ 헤더/라이브러리 설치 (프로덕션 빌드용)
# MuJoCo 3.x 릴리스 tarball 다운로드 후 /opt/mujoco에 배치
# https://github.com/google-deepmind/mujoco/releases

# CMakeLists.txt에 추가할 내용:
# find_package(mujoco REQUIRED)
# target_link_libraries(mujoco_simulator_node mujoco::mujoco)
```

#### 1.2 UR5e MJCF 모델 확보

MuJoCo Menagerie 프로젝트에서 공식 UR5e 모델을 가져온다.

```bash
# 방법 A: MuJoCo Menagerie 클론
git clone https://github.com/google-deepmind/mujoco_menagerie.git /tmp/menagerie
cp -r /tmp/menagerie/universal_robots_ur5e models/ur5e/

# 방법 B: 직접 URDF → MJCF 변환
# ROS2 ur_description 패키지의 URDF를 MuJoCo compile로 변환
```

**`models/ur5e/scene.xml` 구조:**

```xml
<mujoco model="ur5e_scene">
  <option timestep="0.002" gravity="0 0 -9.81"/>  <!-- 500Hz -->

  <include file="ur5e.xml"/>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="2 2 0.1" material="groundplane"/>
    <!-- 로봇은 ur5e.xml에서 포함 -->
  </worldbody>
</mujoco>
```

#### 1.3 관절 이름-인덱스 매핑 확인

UR5e MJCF 관절 이름과 ROS2 `/joint_states` 관절 순서를 정렬:

| 인덱스 | ROS2 joint_names | MuJoCo qpos 인덱스 | MJCF 관절명 |
|--------|------------------|--------------------|------------|
| 0 | `shoulder_pan_joint` | 0 | `shoulder_pan` |
| 1 | `shoulder_lift_joint` | 1 | `shoulder_lift` |
| 2 | `elbow_joint` | 2 | `elbow` |
| 3 | `wrist_1_joint` | 3 | `wrist_1` |
| 4 | `wrist_2_joint` | 4 | `wrist_2` |
| 5 | `wrist_3_joint` | 5 | `wrist_3` |

---

### Phase 2: MuJoCo 래퍼 클래스 구현

**파일:** `include/ur5e_rt_controller/mujoco_simulator.hpp`

```cpp
#pragma once

#include <mujoco/mujoco.h>
#include <array>
#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace urtc {

/// MuJoCo 물리 시뮬레이터 래퍼
/// - 스레드 안전 명령 업데이트 (SetCommand)
/// - 상태 읽기 콜백 (OnStateUpdate)
/// - 선택적 뷰어 스레드
class MuJoCoSimulator {
public:
  struct Config {
    std::string model_path;           // scene.xml 경로
    double      control_freq{500.0};  // 제어 루프 주파수 (Hz)
    bool        enable_viewer{true};  // MuJoCo passive viewer 활성화
    bool        realtime{true};       // 실시간 동기화 여부
    double      sim_speed{1.0};       // 실시간 배율 (2.0 = 2배속)
  };

  /// 상태 업데이트 콜백 타입
  /// (positions, velocities) 각 6개 원소
  using StateCallback = std::function<void(
      const std::array<double, 6>& positions,
      const std::array<double, 6>& velocities)>;

  explicit MuJoCoSimulator(Config cfg) noexcept;
  ~MuJoCoSimulator();

  // 복사/이동 금지
  MuJoCoSimulator(const MuJoCoSimulator&)            = delete;
  MuJoCoSimulator& operator=(const MuJoCoSimulator&) = delete;

  /// 모델 로드 및 시뮬레이션 시작
  [[nodiscard]] bool Initialize() noexcept;

  /// 시뮬레이션 루프 시작 (별도 스레드)
  void Start() noexcept;

  /// 시뮬레이션 정지
  void Stop() noexcept;

  /// 관절 위치 명령 설정 (ROS2 콜백에서 호출)
  /// ctrl 배열에 직접 기록 — 뮤텍스로 보호
  void SetCommand(const std::array<double, 6>& cmd) noexcept;

  /// 상태 업데이트 콜백 등록
  void SetStateCallback(StateCallback cb) noexcept;

  /// 현재 관절 상태 직접 읽기 (폴링 방식 대안)
  [[nodiscard]] std::array<double, 6> GetPositions() const noexcept;
  [[nodiscard]] std::array<double, 6> GetVelocities() const noexcept;

  [[nodiscard]] bool IsRunning() const noexcept { return running_.load(); }
  [[nodiscard]] uint64_t StepCount() const noexcept { return step_count_.load(); }

private:
  Config cfg_;
  mjModel* model_{nullptr};
  mjData*  data_{nullptr};

  std::atomic<bool>     running_{false};
  std::atomic<uint64_t> step_count_{0};

  mutable std::mutex cmd_mutex_;
  std::array<double, 6> pending_cmd_{};
  bool cmd_dirty_{false};

  mutable std::mutex state_mutex_;
  std::array<double, 6> latest_positions_{};
  std::array<double, 6> latest_velocities_{};

  StateCallback state_cb_{nullptr};

  std::jthread sim_thread_;
  std::jthread viewer_thread_;

  void SimLoop(std::stop_token stop) noexcept;
  void ViewerLoop(std::stop_token stop) noexcept;
  void ApplyCommand() noexcept;      // cmd_mutex_ 보호 하에 ctrl 업데이트
  void ReadState() noexcept;         // state_mutex_ 보호 하에 qpos/qvel 복사

  /// 관절 위치 서보 제어 (position control 시뮬레이션)
  /// MuJoCo actuator가 velocity/torque 타입인 경우 내부 PD로 변환
  void ComputeActuatorInput(const std::array<double, 6>& target_pos) noexcept;
};

}  // namespace urtc
```

**MuJoCo 액추에이터 제어 방식:**

UR5e MJCF 모델의 액추에이터 타입에 따라 두 가지 방식 중 선택:

| 방식 | MJCF actuator 타입 | 설명 |
|------|--------------------|------|
| **위치 서보** | `<position>` | `ctrl[i] = target_pos[i]` 직접 설정 (권장) |
| **내부 PD 변환** | `<motor>` or `<velocity>` | `tau = Kp*(q_d - q) + Kd*(dq_d - dq)` 계산 후 토크 입력 |

---

### Phase 3: ROS2 시뮬레이터 노드 구현

**파일:** `src/mujoco_simulator_node.cpp`

#### 3.1 노드 클래스 설계

```cpp
class MuJoCoSimulatorNode : public rclcpp::Node {
public:
  MuJoCoSimulatorNode();

private:
  // MuJoCo 래퍼
  std::unique_ptr<urtc::MuJoCoSimulator> sim_;

  // Publishers (실제 UR 드라이버 역할 대체)
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // Subscriber (custom_controller의 출력 수신)
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      command_sub_;

  // 파라미터
  std::string model_path_;
  bool        enable_viewer_;
  bool        realtime_;
  double      sim_speed_;

  // 관절 이름 (ROS2 /joint_states 메시지에 포함)
  static constexpr std::array<const char*, 6> kJointNames = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};

  void CommandCallback(
      const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void PublishJointState(
      const std::array<double, 6>& pos,
      const std::array<double, 6>& vel);
};
```

#### 3.2 주요 흐름

```
MuJoCoSimulatorNode 초기화
  1. ROS2 파라미터 로드 (model_path, enable_viewer, ...)
  2. MuJoCoSimulator 생성 및 Initialize()
  3. /joint_states publisher 생성
  4. /forward_position_controller/commands subscriber 생성
  5. MuJoCoSimulator.SetStateCallback → PublishJointState 연결
  6. MuJoCoSimulator.Start() → 시뮬레이션 루프 시작

런타임 루프 (시뮬레이션 스레드 내부)
  매 2ms (500Hz):
    1. ApplyCommand() — pending_cmd_를 mj_data.ctrl에 적용
    2. mj_step(model, data) — 물리 스텝 진행
    3. ReadState() — qpos, qvel → latest_positions_, latest_velocities_
    4. state_cb_ 호출 → PublishJointState() → /joint_states 게시

ROS2 콜백 (수신 스레드)
  /forward_position_controller/commands 수신 시:
    1. 데이터 크기 검증 (6개 원소)
    2. sim_.SetCommand(cmd) — cmd_mutex_ 보호
```

---

### Phase 4: 설정 파일 및 런치 파일

#### 4.1 `config/mujoco_simulator.yaml`

```yaml
mujoco_simulator:
  ros__parameters:
    # MuJoCo 모델 경로 (패키지 상대 경로)
    model_path: "models/ur5e/scene.xml"

    # 시뮬레이션 설정
    realtime: true           # false = 가능한 빠르게 실행
    sim_speed: 1.0           # 실시간 배율 (realtime: true일 때)

    # 뷰어
    enable_viewer: true      # MuJoCo GUI 뷰어 활성화

    # 초기 관절 위치 (rad) — 안전 자세
    initial_joint_positions: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

    # 관절 이름 (ROS2 JointState 메시지 순서)
    joint_names:
      - "shoulder_pan_joint"
      - "shoulder_lift_joint"
      - "elbow_joint"
      - "wrist_1_joint"
      - "wrist_2_joint"
      - "wrist_3_joint"
```

#### 4.2 `launch/mujoco_sim.launch.py`

```python
"""
MuJoCo 시뮬레이션 런치 파일

사용법:
  # 기본 (뷰어 활성화)
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py

  # Headless 모드
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py enable_viewer:=false

  # 2배속 시뮬레이션
  ros2 launch ur5e_rt_controller mujoco_sim.launch.py sim_speed:=2.0
"""

런치 노드 목록:
  1. mujoco_simulator_node  ← 시뮬레이터 (UR 드라이버 역할)
  2. custom_controller       ← 기존 제어기 (변경 없음)
  3. motion_editor_gui.py   ← 선택적: 목표 위치 발행 GUI
```

---

### Phase 5: CMakeLists.txt 수정

기존 `CMakeLists.txt`에 추가할 내용:

```cmake
# MuJoCo 의존성 탐색
find_package(mujoco REQUIRED)

# mujoco_simulator_node 실행파일
add_executable(mujoco_simulator_node
  src/mujoco_simulator_node.cpp
)

target_include_directories(mujoco_simulator_node PRIVATE
  include
  ${mujoco_INCLUDE_DIRS}
)

target_link_libraries(mujoco_simulator_node
  ${mujoco_LIBRARIES}
  glfw   # 뷰어 의존성
)

ament_target_dependencies(mujoco_simulator_node
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS mujoco_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

# 모델 파일 설치
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)
```

---

### Phase 6: 핸드 시뮬레이션 (선택적 확장)

`HandState`와 UDP 핸드 프로토콜에 맞춘 확장:

#### 6.1 간이 핸드 모델

```xml
<!-- models/hand/simple_hand.xml -->
<mujoco model="simple_hand">
  <!-- 11개 관절 단순 모델 (sphere geom) -->
  <worldbody>
    <body name="palm">
      <freejoint/>
      <!-- finger_0 ~ finger_10 -->
    </body>
  </worldbody>
</mujoco>
```

#### 6.2 추가 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/hand/joint_states` | `std_msgs/Float64MultiArray` | 게시 | 11개 핸드 관절 상태 |
| `/hand/command` | `std_msgs/Float64MultiArray` | 구독 | 11개 핸드 명령 (0.0–1.0) |

핸드 시뮬레이션은 실제 물리 없이 **단순 1차 저역 필터**로도 대체 가능:

```cpp
// hand_state[i] += alpha * (hand_cmd[i] - hand_state[i])
// alpha ≈ 0.1 (100Hz에서 ~10ms 시정수)
```

---

## 5. 테스트 계획

### 5.1 단계별 검증

| 단계 | 테스트 항목 | 성공 기준 |
|------|-------------|-----------|
| Phase 2 | MuJoCo 로드 및 `mj_step()` 실행 | 크래시 없이 1000스텝 진행 |
| Phase 3 | `/joint_states` 게시 확인 | `ros2 topic hz /joint_states` → ~500Hz |
| Phase 3 | 명령 수신 → 시뮬레이터 반영 | 관절이 목표 위치로 이동 |
| Phase 3 | E-STOP 동작 | `enable_estop: true` 시 안전 자세로 복귀 |
| Phase 4 | 핸드 상태 게시 | `/hand/joint_states` @ 100Hz |

### 5.2 시뮬레이션 vs 실제 비교 지표

- 관절 추종 오차 (RMS): 목표 < 0.01 rad
- 제어 루프 지터: < 100μs (실시간 미적용이므로 RT보다 완화)
- 500Hz 유지율: > 99%

### 5.3 실행 명령 예시

```bash
# 터미널 1: MuJoCo 시뮬레이터 + 제어기 통합 실행
ros2 launch ur5e_rt_controller mujoco_sim.launch.py

# 터미널 2: 목표 관절 위치 발행
ros2 topic pub /target_joint_positions std_msgs/msg/Float64MultiArray \
  "data: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]"

# 터미널 3: 상태 모니터링
ros2 topic hz /joint_states
ros2 topic echo /system/estop_status

# 터미널 4: 궤적 시각화 (기존 스크립트 재활용)
ros2 run ur5e_rt_controller plot_ur_trajectory.py /tmp/ur5e_control_log.csv
```

---

## 6. 의존성 요약

| 패키지 | 버전 | 용도 |
|--------|------|------|
| `mujoco` | ≥ 3.0.0 | 물리 엔진 C++ 라이브러리 |
| `glfw3` | ≥ 3.3 | MuJoCo 뷰어 윈도우 |
| `ur5e MJCF` | MuJoCo Menagerie | UR5e 로봇 모델 파일 |

```bash
# 설치 명령
sudo apt install libglfw3-dev
pip install mujoco  # Python 검증용

# MuJoCo C++ 라이브러리 (시스템 설치)
wget https://github.com/google-deepmind/mujoco/releases/download/3.x.x/mujoco-3.x.x-linux-x86_64.tar.gz
sudo tar -xzf mujoco-*.tar.gz -C /opt/
echo 'export MUJOCO_DIR=/opt/mujoco-3.x.x' >> ~/.bashrc
```

---

## 7. 구현 우선순위 및 일정

| 우선순위 | Phase | 주요 작업 | 예상 공수 |
|----------|-------|-----------|-----------|
| 1 (필수) | 1 | 환경 구성, UR5e MJCF 모델 확보 | 소 |
| 2 (필수) | 2 | `mujoco_simulator.hpp` 구현 | 중 |
| 3 (필수) | 3 | `mujoco_simulator_node.cpp` + ROS2 브릿지 | 중 |
| 4 (필수) | 4 | 설정 파일 + 런치 파일 | 소 |
| 5 (필수) | 5 | `CMakeLists.txt` 수정, 빌드 검증 | 소 |
| 6 (선택) | 6 | 핸드 시뮬레이션 추가 | 중 |

---

## 8. 주요 설계 결정 및 트레이드오프

### 8.1 C++ vs Python 구현

| 항목 | C++ 노드 | Python 노드 |
|------|----------|-------------|
| 성능 | 높음 (native) | 낮음 (GIL) |
| 개발 속도 | 느림 | 빠름 |
| 기존 코드 통합 | 자연스러움 | 별도 프로세스 |
| 권장 용도 | 프로덕션 | 프로토타입 |

**결정: C++ 구현 채택** — 기존 코드베이스와 일관성 유지, RT 특성 분석 가능

### 8.2 MuJoCo 액추에이터 타입

**`<position>` 서보 액추에이터 채택:**
- `ctrl[i] = target_position[i]` 직접 설정
- MuJoCo 내부 PD 게인으로 추종
- `/forward_position_controller/commands`의 위치 명령과 자연스럽게 대응

### 8.3 시뮬레이션 시간 동기화

- `realtime: true`: `nanosleep()`으로 벽시계 동기화 → 제어 주기 분석에 적합
- `realtime: false`: 최대 속도로 실행 → 알고리즘 검증, 데이터 수집에 적합

---

## 9. 참고 자료

- [MuJoCo 공식 문서](https://mujoco.readthedocs.io/)
- [MuJoCo Menagerie — UR5e 모델](https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e)
- [MuJoCo C API 레퍼런스](https://mujoco.readthedocs.io/en/stable/APIreference/)
- [ROS2 + MuJoCo 통합 패턴](https://github.com/google-deepmind/mujoco/blob/main/doc/programming/index.rst)
- 기존 코드: `include/ur5e_rt_controller/rt_controller_interface.hpp` (데이터 구조)
- 기존 코드: `src/custom_controller.cpp` (제어기 아키텍처)
