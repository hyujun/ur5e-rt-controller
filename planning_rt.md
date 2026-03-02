# RT 스레드 설정 및 동기화 수정 계획

**작성일**: 2026-03-02
**대상 버전**: v4.2.2
**목표**: 검토에서 발견된 10개 이슈를 심각도 순으로 수정하여 진정한 RT 안전성 확보

---

## 전체 수정 요약

| # | 심각도 | 파일 | 문제 | 수정 방향 |
|---|---|---|---|---|
| 1 | CRITICAL | `custom_controller.cpp:258` | RT 스레드에서 DataLogger 직접 호출 | lock-free 링 버퍼로 로그 데이터 넘기기 |
| 2 | CRITICAL | `custom_controller.cpp:235,304` | `state_received_`/`target_received_` 데이터 레이스 | `std::atomic<bool>` 으로 교체 |
| 3 | CRITICAL | `custom_controller.cpp:184` | 뮤텍스 해제 후 `target_positions_` 읽기 | 뮤텍스 안에서 복사본을 만들어 전달 |
| 4 | MAJOR | `custom_controller.cpp:247,260` | `target_mutex_` 블록 논리 오류 + 로깅 시 무보호 읽기 | 블록 재구성 및 지역 복사본 사용 |
| 5 | MAJOR | `custom_controller.cpp:253` | RT 스레드에서 `publish()` 힙 할당 | `RealtimePublisher` 또는 pre-allocated 메시지 |
| 6 | MAJOR | `hand_udp_receiver.hpp/.cpp` | jthread에 RT 설정 미적용 | `ReceiveLoop` 진입 시 `ApplyThreadConfig` 호출 |
| 7 | MAJOR | `custom_controller.cpp:324` | `mlockall`이 `rclcpp::init` 이후 호출 | 호출 순서 역전 |
| 8 | MEDIUM | `thread_utils.hpp:112` | `GetThreadStats` include 누락 | 필요한 헤더 추가 |
| 9 | LOW | `thread_config.hpp`, `custom_controller.cpp` | 4코어 fallback 자동 미적용 | 런타임 코어 수 감지 함수 추가 |

---

## 수정 1: RT 스레드에서 DataLogger 분리 [CRITICAL]

### 문제

`ControlLoop()`(SCHED_FIFO 90, Core 2)에서 `logger_->LogControlData()`를 직접 호출한다.
`std::ofstream` 쓰기는 커널 syscall이며 수백 µs ~ 수 ms 블로킹이 발생할 수 있어 500Hz 주기를 파괴한다.

```cpp
// 현재 — 잘못된 코드 (custom_controller.cpp:258)
if (enable_logging_ && logger_) {
    logger_->LogControlData(now().seconds(),          // syscall 포함 가능
                            state.robot.positions,
                            target_positions_,
                            output.robot_commands);   // 파일 I/O in RT thread!
}
```

### 해결 방안

**lock-free 단일 생산자/단일 소비자(SPSC) 링 버퍼**를 도입한다.

- RT 스레드(생산자): 로그 엔트리를 링 버퍼에 복사 → O(1), 힙 할당·syscall 없음
- log 스레드(소비자, Core 4): 링 버퍼에서 꺼내 파일에 씀

### 수정 대상 파일

- `include/ur5e_rt_controller/log_buffer.hpp` (신규)
- `include/ur5e_rt_controller/data_logger.hpp` (수정)
- `src/custom_controller.cpp` (수정)

### 상세 구현

#### A. `log_buffer.hpp` 신규 생성

```cpp
// include/ur5e_rt_controller/log_buffer.hpp
#pragma once
#include "ur5e_rt_controller/rt_controller_interface.hpp"

#include <array>
#include <atomic>
#include <cstddef>

namespace ur5e_rt_controller {

// RT 안전 SPSC 링 버퍼 — 로그 엔트리 전용
// 생산자(RT 스레드)와 소비자(log 스레드) 각각 하나씩만 허용
struct LogEntry {
  double timestamp;
  std::array<double, kNumRobotJoints> current_positions;
  std::array<double, kNumRobotJoints> target_positions;
  std::array<double, kNumRobotJoints> commands;
};

template <std::size_t N>
class SpscLogBuffer {
 public:
  // RT 스레드에서 호출 — 힙 할당 없음, lock-free
  // 버퍼가 가득 차면 false 반환 (드롭, 로그 누락 허용)
  [[nodiscard]] bool Push(const LogEntry& entry) noexcept {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) % N;
    if (next == tail_.load(std::memory_order_acquire)) {
      return false;  // 버퍼 가득 참 — 드롭
    }
    buffer_[head] = entry;
    head_.store(next, std::memory_order_release);
    return true;
  }

  // log 스레드에서 호출
  [[nodiscard]] bool Pop(LogEntry& out) noexcept {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      return false;  // 비어 있음
    }
    out = buffer_[tail];
    tail_.store((tail + 1) % N, std::memory_order_release);
    return true;
  }

 private:
  std::array<LogEntry, N> buffer_{};
  // head: 생산자(RT)가 씀 / tail: 소비자(log)가 씀
  alignas(64) std::atomic<std::size_t> head_{0};  // 캐시라인 분리
  alignas(64) std::atomic<std::size_t> tail_{0};
};

// 500Hz × 최대 1초치 여유 = 512 (2의 제곱수)
using ControlLogBuffer = SpscLogBuffer<512>;

}  // namespace ur5e_rt_controller
```

#### B. `DataLogger` 수정 — log 스레드 전용 드레인 메서드 추가

```cpp
// include/ur5e_rt_controller/data_logger.hpp 에 추가
void DrainBuffer(ControlLogBuffer& buf);  // log 스레드에서 주기적으로 호출
```

#### C. `custom_controller.cpp` 수정

```cpp
// 멤버 변수 추가
urtc::ControlLogBuffer log_buffer_;

// ControlLoop() — RT 스레드 (수정 후)
void ControlLoop() {
    ...
    const urtc::ControllerOutput output = controller_->Compute(state);
    // publish (수정 5 이후 RT-safe publisher 사용)
    PublishCommand(output);

    // 로깅: 링 버퍼에만 넣고 즉시 반환 — 파일 I/O 없음
    if (enable_logging_) {
        urtc::LogEntry entry{
            .timestamp        = now().seconds(),
            .current_positions = state.robot.positions,
            .target_positions  = target_snapshot_,   // 수정 4에서 도입하는 복사본
            .commands          = output.robot_commands,
        };
        log_buffer_.Push(entry);  // 가득 차면 드롭 (RT 안전)
    }
    ...
}

// DrainLog() — log 스레드의 타이머 콜백 (신규)
void DrainLog() {
    if (!logger_) return;
    logger_->DrainBuffer(log_buffer_);  // 파일 쓰기는 Core 4에서만
}
```

`cb_group_log_`에 100Hz 타이머를 추가하여 `DrainLog()`를 호출한다:
```cpp
// CreateTimers()에 추가
drain_timer_ = create_wall_timer(
    10ms,                          // 100Hz
    [this]() { DrainLog(); },
    cb_group_log_);
```

---

## 수정 2: `state_received_` / `target_received_` 데이터 레이스 [CRITICAL]

### 문제

```cpp
// 선언 (custom_controller.cpp:304-306)
bool state_received_{false};   // plain bool — atomic 아님
bool target_received_{false};  // plain bool — atomic 아님
bool hand_data_received_{false};

// ControlLoop() — 뮤텍스 없이 읽음 (RT 스레드)
if (!state_received_ || !target_received_) { return; }

// JointStateCallback() — state_mutex_ 안에서 씀 (sensor 스레드)
state_received_ = true;
```

두 스레드에서 동시 접근 → C++ 메모리 모델상 undefined behavior.

### 수정 방법

`std::atomic<bool>`로 교체. 쓰기는 `release`, 읽기는 `acquire` 사용.

```cpp
// 수정 후 선언
std::atomic<bool> state_received_{false};
std::atomic<bool> target_received_{false};
std::atomic<bool> hand_data_received_{false};

// JointStateCallback() — 뮤텍스 안에서 store
{
    std::lock_guard lock(state_mutex_);
    std::copy_n(...);
    last_robot_update_ = now();
    state_received_.store(true, std::memory_order_release);
}

// TargetCallback()
{
    std::lock_guard lock(target_mutex_);
    std::copy_n(...);
    target_received_.store(true, std::memory_order_release);
}

// HandStateCallback()
{
    std::lock_guard lock(hand_mutex_);
    last_hand_update_ = now();
    hand_data_received_.store(true, std::memory_order_release);
}

// ControlLoop() — 뮤텍스 없이 안전하게 읽기
if (!state_received_.load(std::memory_order_acquire) ||
    !target_received_.load(std::memory_order_acquire)) {
    return;
}

// CheckTimeouts() — hand_data_received_ 읽기도 수정
{
    std::lock_guard lock(hand_mutex_);
    if (hand_data_received_.load(std::memory_order_relaxed)) {
        hand_timed_out = (now_time - last_hand_update_) > hand_timeout_;
    }
}
```

**주의**: `state_received_`가 `state_mutex_` 안에서 설정되더라도, `ControlLoop`에서 뮤텍스 없이 읽으려면 atomic이어야 한다. mutex는 mutex 안의 데이터만 보호한다.

---

## 수정 3: `TargetCallback`에서 뮤텍스 해제 후 읽기 [CRITICAL]

### 문제

```cpp
// 현재 — 잘못된 코드 (custom_controller.cpp:178-184)
void TargetCallback(...) {
    {
        std::lock_guard lock(target_mutex_);
        std::copy_n(msg->data.begin(), kNumRobotJoints, target_positions_.begin());
        target_received_.store(true, std::memory_order_release);
    }  // ← 뮤텍스 해제
    controller_->SetRobotTarget(target_positions_);  // ← 무보호 읽기!
}
```

뮤텍스 해제 후 `target_positions_`를 읽는 사이에 RT 스레드가 동일 배열을 수정할 수 있다.

### 수정 방법

뮤텍스 안에서 `std::span`을 직접 전달하거나, 지역 복사본을 만들어 뮤텍스 밖에서 사용한다.

```cpp
// 수정 후 — 지역 복사본 사용
void TargetCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < urtc::kNumRobotJoints) {
        return;
    }

    std::array<double, urtc::kNumRobotJoints> local_target;
    {
        std::lock_guard lock(target_mutex_);
        std::copy_n(msg->data.begin(), urtc::kNumRobotJoints, target_positions_.begin());
        local_target = target_positions_;  // 뮤텍스 안에서 복사본 생성
        target_received_.store(true, std::memory_order_release);
    }
    controller_->SetRobotTarget(local_target);  // 복사본으로 호출 — 안전
}
```

---

## 수정 4: `target_mutex_` 블록 논리 오류 및 로깅 무보호 읽기 [MAJOR]

### 문제 A — `target_mutex_` 블록이 관계없는 필드를 보호

```cpp
// 현재 — 잘못된 코드 (custom_controller.cpp:246-249)
{
    std::lock_guard lock(target_mutex_);
    state.robot.dt   = 1.0 / control_rate_;  // target_positions_와 무관
    state.iteration  = loop_count_;           // target_positions_와 무관
    // target_positions_를 여기서 복사하지 않음!
}
```

### 문제 B — 로깅 시 `target_positions_` 무보호 읽기

```cpp
// 현재 — 잘못된 코드 (custom_controller.cpp:259-263)
logger_->LogControlData(...,
    target_positions_,  // ← 뮤텍스 없이 읽음
    ...);
```

### 해결 방안

`target_snapshot_` 멤버 변수를 도입하여 `ControlLoop` 진입 시 한 번 복사하고 이후 해당 복사본만 사용한다.

```cpp
// 멤버 변수 추가
std::array<double, urtc::kNumRobotJoints> target_snapshot_{};

// 수정 후 ControlLoop()
void ControlLoop() {
    if (!state_received_.load(std::memory_order_acquire) ||
        !target_received_.load(std::memory_order_acquire)) {
        return;
    }

    urtc::ControllerState state{};

    // 상태 스냅샷
    {
        std::lock_guard lock(state_mutex_);
        state.robot.positions  = current_positions_;
        state.robot.velocities = current_velocities_;
    }

    // 타겟 스냅샷 — target_mutex_ 안에서 복사 후 복사본만 사용
    {
        std::lock_guard lock(target_mutex_);
        target_snapshot_ = target_positions_;  // 여기서 복사
    }

    state.robot.dt  = 1.0 / control_rate_;  // 뮤텍스 불필요
    state.iteration = loop_count_;           // 뮤텍스 불필요

    const urtc::ControllerOutput output = controller_->Compute(state);
    PublishCommand(output);

    if (enable_logging_) {
        urtc::LogEntry entry{
            .timestamp         = now().seconds(),
            .current_positions = state.robot.positions,
            .target_positions  = target_snapshot_,  // 복사본 사용 — 안전
            .commands          = output.robot_commands,
        };
        log_buffer_.Push(entry);
    }

    ++loop_count_;
}
```

---

## 수정 5: RT 스레드에서 `publish()` — `RealtimePublisher` 도입 [MAJOR]

### 문제

```cpp
// 현재 (custom_controller.cpp:253-256) — RT 스레드에서 DDS 힙 할당 가능
std_msgs::msg::Float64MultiArray cmd_msg;
cmd_msg.data.assign(...);   // vector realloc 가능
command_pub_->publish(cmd_msg);
```

### 해결 방안

`realtime_tools::RealtimePublisher<T>`를 사용한다. 이 클래스는 내부적으로 pre-allocated 메시지를 갖고 trylock 방식으로 발행하여 RT 스레드 블로킹을 방지한다.

#### A. 멤버 변수 교체

```cpp
// 기존
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

// 수정 후
std::unique_ptr<realtime_tools::RealtimePublisher<
    std_msgs::msg::Float64MultiArray>> rt_command_pub_;
```

#### B. `CreatePublishers()` 수정

```cpp
void CreatePublishers() {
    auto raw_pub = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_position_controller/commands", 10);
    rt_command_pub_ = std::make_unique<
        realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(raw_pub);

    // msg.data를 미리 할당 — RT 루프에서 resize 없음
    rt_command_pub_->msg_.data.resize(urtc::kNumRobotJoints, 0.0);

    estop_pub_ = create_publisher<std_msgs::msg::Bool>("/system/estop_status", 10);
}
```

#### C. RT 루프에서 trylock 발행

```cpp
// ControlLoop()에서
void PublishCommand(const urtc::ControllerOutput& output) {
    if (rt_command_pub_->trylock()) {
        std::copy(output.robot_commands.begin(),
                  output.robot_commands.end(),
                  rt_command_pub_->msg_.data.begin());
        rt_command_pub_->unlockAndPublish();
    }
    // trylock 실패(소비자가 아직 처리 중)는 허용 — 해당 사이클 드롭
}
```

**E-STOP 퍼블리시**: `estop_pub_`는 50Hz watchdog에서만 호출되고 빈도가 낮으므로 현재 방식을 유지해도 무방하다. 단, 동일하게 `RealtimePublisher`로 교체하면 더 일관된다.

---

## 수정 6: `HandUdpReceiver` jthread에 RT 설정 적용 [MAJOR]

### 문제

`kUdpRecvConfig` (Core 3, SCHED_FIFO 65)가 선언만 되어 있고, `HandUdpReceiver`의 `jthread`에 전혀 적용되지 않는다.

### 수정 대상 파일

- `include/ur5e_rt_controller/hand_udp_receiver.hpp`
- `src/hand_udp_receiver_node.cpp`

#### A. `HandUdpReceiver`에 `ThreadConfig` 파라미터 추가

```cpp
// hand_udp_receiver.hpp 수정
#include "ur5e_rt_controller/thread_config.hpp"
#include "ur5e_rt_controller/thread_utils.hpp"

class HandUdpReceiver {
 public:
  // 기본값으로 kUdpRecvConfig 사용 — 기존 호출부 호환 유지
  explicit HandUdpReceiver(int port,
      const ThreadConfig& thread_cfg = kUdpRecvConfig) noexcept;
  ...
 private:
  ThreadConfig thread_cfg_;  // 추가
  void ReceiveLoop(std::stop_token stop_token);
};
```

#### B. `ReceiveLoop` 진입 시 `ApplyThreadConfig` 호출

```cpp
// hand_udp_receiver.cpp (또는 구현부)
void HandUdpReceiver::ReceiveLoop(std::stop_token stop_token) {
    // jthread 시작 직후 즉시 RT 설정 적용
    if (!ApplyThreadConfig(thread_cfg_)) {
        // 실패해도 계속 — RT 권한 없는 환경 허용
        fprintf(stderr, "[WARN] HandUdpReceiver: thread config failed\n");
    }

    while (!stop_token.stop_requested()) {
        // ... 기존 수신 로직
    }
}
```

#### C. `hand_udp_receiver_node.cpp` — mlockall 추가

`HandUdpReceiverNode`는 별도 프로세스이므로 자체적으로 `mlockall`이 필요하다:

```cpp
int main(int argc, char** argv) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        fprintf(stderr, "[WARN] mlockall failed\n");
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandUdpReceiverNode>());
    rclcpp::shutdown();
    return 0;
}
```

---

## 수정 7: `mlockall` 호출 순서 수정 [MAJOR]

### 문제

```cpp
// 현재 (custom_controller.cpp:324-331)
rclcpp::init(argc, argv);          // 1. ROS2/DDS 초기화 — 이 시점에 힙/스택 할당 발생
if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {  // 2. 이미 늦음
    fprintf(stderr, "[WARN] mlockall failed\n");
}
auto node = std::make_shared<CustomController>();
```

`MCL_CURRENT`는 호출 시점의 메모리만 잠그므로, `rclcpp::init`에서 DDS가 할당한 페이지들은 잠기지 않는다.

### 수정 방법

```cpp
// 수정 후 — main()의 첫 번째 작업으로 mlockall 실행
int main(int argc, char** argv) {
    // 1. 메모리 잠금 — 모든 초기화 전에 실행 (MCL_CURRENT는 여기까지의 페이지 잠금)
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        fprintf(stderr, "[WARN] mlockall failed — page faults possible\n");
        fprintf(stderr, "       Check: /etc/security/limits.conf @realtime memlock unlimited\n");
    }

    // 2. ROS2 초기화 — MCL_FUTURE 덕분에 이후 할당도 자동 잠금
    rclcpp::init(argc, argv);

    // 3. 노드 생성 및 실행
    auto node = std::make_shared<CustomController>();
    ...
}
```

---

## 수정 8: `GetThreadStats` 누락 include 추가 [MEDIUM]

### 문제

`thread_utils.hpp`가 `std::min_element`, `std::max_element` (`<algorithm>`),
`std::accumulate` (`<numeric>`), `std::vector` (`<vector>`), `std::tuple` (`<tuple>`)을
사용하지만 해당 헤더를 포함하지 않는다.

### 수정 방법

```cpp
// thread_utils.hpp — include 블록에 추가
#include <algorithm>    // std::min_element, std::max_element
#include <numeric>      // std::accumulate
#include <tuple>        // std::tuple
#include <vector>       // std::vector
```

---

## 수정 9: 4코어 시스템 자동 fallback [LOW]

### 문제

`kRtControlConfig4Core` 등이 선언되어 있지만 `custom_controller.cpp`는 항상 6코어 설정을 하드코딩한다. 4코어 시스템에서 실행하면 `pthread_setaffinity_np`가 실패하고 RT 없이 실행된다.

### 해결 방안

`thread_utils.hpp`에 헬퍼 함수를 추가하고 `main()`에서 이를 사용한다.

#### A. `thread_utils.hpp`에 감지 함수 추가

```cpp
// thread_utils.hpp에 추가
#include <unistd.h>  // 이미 포함됨

// 현재 시스템의 온라인 CPU 수 반환
inline int GetOnlineCpuCount() noexcept {
    const int n = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    return (n > 0) ? n : 1;
}
```

#### B. `thread_config.hpp`에 선택 함수 추가

```cpp
// thread_config.hpp에 추가
#include "ur5e_rt_controller/thread_utils.hpp"  // GetOnlineCpuCount

struct SystemThreadConfigs {
    ThreadConfig rt_control;
    ThreadConfig sensor;
    ThreadConfig logging;
    ThreadConfig aux;
};

inline SystemThreadConfigs SelectThreadConfigs() noexcept {
    const int cores = GetOnlineCpuCount();
    if (cores >= 6) {
        return {kRtControlConfig, kSensorConfig, kLoggingConfig, kAuxConfig};
    } else {
        // 4코어 fallback — aux는 logging과 같은 코어 사용
        return {kRtControlConfig4Core, kSensorConfig4Core,
                kLoggingConfig4Core, kLoggingConfig4Core};
    }
}
```

#### C. `main()`에서 동적 선택

```cpp
// custom_controller.cpp main() 수정
int main(int argc, char** argv) {
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CustomController>();
    ...

    const auto cfgs = urtc::SelectThreadConfigs();

    auto t_rt     = make_thread(rt_executor,     cfgs.rt_control);
    auto t_sensor = make_thread(sensor_executor, cfgs.sensor);
    auto t_log    = make_thread(log_executor,    cfgs.logging);
    auto t_aux    = make_thread(aux_executor,    cfgs.aux);
    ...
}
```

---

## 수정 적용 순서 (의존성 고려)

```
수정 7 (mlockall 순서)          ← 독립적, 가장 먼저 적용 (1줄 변경)
수정 8 (include 추가)           ← 독립적, 즉시 적용
수정 2 (atomic<bool>)           ← 수정 3, 4 이전에 적용 (선언 변경)
수정 3 (TargetCallback 복사본)  ← 수정 2 이후
수정 1 (링 버퍼 + DrainLog)    ← 수정 4와 함께 (log_buffer_ 공유)
수정 4 (target_snapshot_)       ← 수정 1, 2, 3 이후
수정 5 (RealtimePublisher)      ← 수정 4 이후 (ControlLoop 구조 안정화 후)
수정 6 (jthread RT 설정)        ← 독립적 (다른 파일)
수정 9 (4코어 fallback)         ← 마지막 (선택적)
```

---

## 검증 체크리스트

수정 완료 후 아래 항목을 확인한다.

### 빌드 검증
```bash
cd ~/ur_ws
colcon build --packages-select ur5e_rt_controller --symlink-install
# 경고 없이 빌드 완료 확인
```

### 스레드 설정 확인
```bash
ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true &
sleep 3
PID=$(pgrep -f custom_controller)
ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
# 기대 출력:
# XXXX  YYYY  FF  90  2  rt_control   ← Core 2, SCHED_FIFO 90
# XXXX  YYYY  FF  70  3  sensor_io    ← Core 3, SCHED_FIFO 70
# XXXX  YYYY  TS   -  4  logger       ← Core 4, SCHED_OTHER
# XXXX  YYYY  TS   -  5  aux          ← Core 5, SCHED_OTHER
```

### 500Hz 주기 안정성 확인
```bash
ros2 topic hz /forward_position_controller/commands
# 기대: average rate: 500.0 ± 0.5, std dev < 0.000050s (50μs)
```

### RT 지터 측정 (cyclictest)
```bash
sudo cyclictest --mlockall --smp --priority=90 --policy=fifo \
    --interval=2000 --loops=100000 --affinity=2
# 기대: Max jitter < 50μs
```

### 데이터 레이스 검사 (ThreadSanitizer)
```bash
# CMakeLists.txt에 임시 추가:
# add_compile_options(-fsanitize=thread)
# add_link_options(-fsanitize=thread)
colcon build --packages-select ur5e_rt_controller
ros2 launch ur5e_rt_controller ur_control.launch.py use_fake_hardware:=true
# TSAN 경고 없음 확인
```

### 로그 파일 생성 확인
```bash
# 수정 1 이후 로그가 log 스레드에서 정상 작성되는지 확인
tail -f /tmp/ur5e_control_log.csv
# 데이터가 ~100Hz로 추가되면 정상
```

### mlockall 확인
```bash
PID=$(pgrep -f custom_controller)
grep -c "^[0-9a-f]" /proc/$PID/smaps | head -1
# VmLck 항목 확인
grep VmLck /proc/$PID/status
# VmLck > 0 이면 잠금 성공
```

---

## 수정 후 예상 효과

| 지표 | 수정 전 | 수정 후 (예상) |
|---|---|---|
| RT 루프 지터 (max) | ~500μs (파일 I/O 포함) | <50μs |
| 데이터 레이스 | 3개 UB 존재 | 0 |
| `mlockall` 효과 | DDS 스택 미잠금 | 전체 프로세스 잠금 |
| UDP recv RT 설정 | 미적용 | Core 3, FIFO 65 |
| 4코어 시스템 | affinity 실패 후 비RT | 자동 fallback |
