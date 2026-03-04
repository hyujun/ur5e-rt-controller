#ifndef UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_
#define UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_

// ── Includes: project, then MuJoCo, then C++ stdlib ───────────────────────────
#include <mujoco/mujoco.h>

#ifdef MUJOCO_HAVE_GLFW
#include <GLFW/glfw3.h>
#endif

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace ur5e_rt_controller {

// ── MuJoCoSimulator ────────────────────────────────────────────────────────────
//
// Thread-safe wrapper around a MuJoCo physics model.
//
// Simulation modes:
//   kFreeRun  — advances mj_step() as fast as possible (up to max_rtf).
//               Best for algorithm validation, trajectory generation.
//   kSyncStep — publishes state, waits for one controller command, then takes
//               one physics step.  Step latency ≈ controller Compute() time.
//
// Runtime controls (thread-safe, call from any thread):
//   Pause()        — freeze physics without stopping the viewer
//   Resume()       — resume physics
//   RequestReset() — reinitialise to cfg_.initial_qpos (sim thread picks it up)
//   SetMaxRtf()    — adjust speed cap at runtime (+/- keys in viewer)
//
// Viewer keyboard shortcuts (MUJOCO_HAVE_GLFW only):
//   Space         — pause / resume
//   + / KP_ADD   — double max_rtf (speed up; unlimited → 2x)
//   - / KP_SUB   — halve max_rtf (slow down; ≤0.5x → unlimited)
//   R             — reset simulation to initial pose
//   F3            — toggle RTF profiler graph
//   Backspace     — reset visualisation options
//   Escape        — reset camera to default position
//
// Viewer mouse controls:
//   Left drag     — orbit (rotate) camera
//   Right drag    — pan (translate) camera
//   Scroll        — zoom in / out
//
// Threading model:
//   SimLoop thread  — runs SimLoopFreeRun or SimLoopSyncStep.
//   ViewerLoop thread — renders scene at ~60 Hz via GLFW (optional).
//   Caller thread   — calls SetCommand(), GetPositions(), GetVelocities().
//
// Synchronisation:
//   cmd_mutex_   — protects pending_cmd_ (caller → sim write)
//   cmd_pending_ — atomic flag for lock-free fast-path in FreeRun mode
//   sync_cv_     — condition variable to wake SimLoopSyncStep on command
//   state_mutex_ — protects latest_positions_ / latest_velocities_
//   viz_mutex_   — protects viz_qpos_ / viz_ncon_ (try_lock avoids blocking SimLoop)
//
class MuJoCoSimulator {
 public:
  enum class SimMode {
    kFreeRun,   // Maximum speed (throttled by max_rtf)
    kSyncStep,  // 1:1 synchronised with controller commands
  };

  struct Config {
    std::string model_path;
    SimMode     mode{SimMode::kFreeRun};
    bool        enable_viewer{true};
    int         publish_decimation{1};   // kFreeRun: publish every N steps
    double      sync_timeout_ms{50.0};   // kSyncStep: command wait timeout
    double      max_rtf{0.0};           // 0.0 = unlimited
    std::array<double, 6> initial_qpos{
        0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};
  };

  // Invoked from SimLoop after each publish step.
  using StateCallback = std::function<void(
      const std::array<double, 6>& positions,
      const std::array<double, 6>& velocities)>;

  explicit MuJoCoSimulator(Config cfg) noexcept;
  ~MuJoCoSimulator();

  MuJoCoSimulator(const MuJoCoSimulator&)            = delete;
  MuJoCoSimulator& operator=(const MuJoCoSimulator&) = delete;
  MuJoCoSimulator(MuJoCoSimulator&&)                 = delete;
  MuJoCoSimulator& operator=(MuJoCoSimulator&&)      = delete;

  // Load MJCF model and resolve joint indices.  Must be called before Start().
  [[nodiscard]] bool Initialize() noexcept;

  // Start simulation and viewer threads.
  void Start() noexcept;

  // Signal stop and join all threads.
  void Stop() noexcept;

  // Write a position command into the pending buffer.
  // Thread-safe.  In kSyncStep mode also wakes the simulation loop.
  void SetCommand(const std::array<double, 6>& cmd) noexcept;

  // Register the callback invoked after each publish step.
  void SetStateCallback(StateCallback cb) noexcept;

  [[nodiscard]] std::array<double, 6> GetPositions()  const noexcept;
  [[nodiscard]] std::array<double, 6> GetVelocities() const noexcept;

  // ── Runtime controls (thread-safe) ────────────────────────────────────────
  void Pause()   noexcept { paused_.store(true,  std::memory_order_relaxed); }
  void Resume()  noexcept { paused_.store(false, std::memory_order_relaxed); sync_cv_.notify_all(); }
  [[nodiscard]] bool IsPaused() const noexcept { return paused_.load(std::memory_order_relaxed); }

  // Request physics reset to cfg_.initial_qpos (executed on the sim thread).
  void RequestReset() noexcept { reset_requested_.store(true, std::memory_order_relaxed); sync_cv_.notify_all(); }

  // Set maximum Real-Time Factor at runtime (0.0 = unlimited).
  void SetMaxRtf(double rtf) noexcept {
    current_max_rtf_.store(rtf < 0.0 ? 0.0 : rtf, std::memory_order_relaxed);
  }
  [[nodiscard]] double GetMaxRtf() const noexcept {
    return current_max_rtf_.load(std::memory_order_relaxed);
  }

  // ── Status accessors ──────────────────────────────────────────────────────
  [[nodiscard]] bool     IsRunning()  const noexcept { return running_.load(); }
  [[nodiscard]] uint64_t StepCount()  const noexcept { return step_count_.load(); }
  [[nodiscard]] double   SimTimeSec() const noexcept { return sim_time_sec_.load(); }
  [[nodiscard]] int      NumJoints()  const noexcept { return model_ ? model_->nq : 0; }
  // Real-Time Factor: sim_time / wall_time (updated every 200 steps).
  [[nodiscard]] double   GetRtf()     const noexcept { return rtf_.load(std::memory_order_relaxed); }

 private:
  Config   cfg_;
  mjModel* model_{nullptr};
  mjData*  data_{nullptr};

  std::atomic<bool>     running_{false};
  std::atomic<uint64_t> step_count_{0};
  std::atomic<double>   sim_time_sec_{0.0};

  // ── Runtime control flags ─────────────────────────────────────────────────
  std::atomic<bool>   paused_{false};
  std::atomic<bool>   reset_requested_{false};
  std::atomic<double> current_max_rtf_{0.0};  // runtime-adjustable speed cap

  // ── Command buffer ────────────────────────────────────────────────────────
  mutable std::mutex    cmd_mutex_;
  std::atomic<bool>     cmd_pending_{false};
  std::array<double, 6> pending_cmd_{};

  // SyncStep synchronisation — woken by SetCommand(), Resume(), RequestReset(), Stop().
  std::mutex              sync_mutex_;
  std::condition_variable sync_cv_;

  // ── State buffer ──────────────────────────────────────────────────────────
  mutable std::mutex    state_mutex_;
  std::array<double, 6> latest_positions_{};
  std::array<double, 6> latest_velocities_{};

  // ── Viewer double-buffer ──────────────────────────────────────────────────
  mutable std::mutex  viz_mutex_;
  std::vector<double> viz_qpos_{};
  int                 viz_ncon_{0};   // contact count snapshot for overlay
  bool                viz_dirty_{false};

  StateCallback state_cb_{nullptr};

  std::jthread sim_thread_;
  std::jthread viewer_thread_;

  // Joint index maps resolved from MJCF joint names at initialise time.
  std::array<int, 6> joint_qpos_indices_{0, 1, 2, 3, 4, 5};
  std::array<int, 6> joint_qvel_indices_{0, 1, 2, 3, 4, 5};

  // ── RTF measurement ───────────────────────────────────────────────────────
  std::chrono::steady_clock::time_point rtf_wall_start_{};
  double                                rtf_sim_start_{0.0};
  std::atomic<double>                   rtf_{0.0};

  // ── Max-RTF throttle (sim thread only) ───────────────────────────────────
  std::chrono::steady_clock::time_point throttle_wall_start_{};
  double                                throttle_sim_start_{0.0};
  double                                throttle_rtf_{0.0};  // last applied value (for change detection)

  // ── Internal helpers ───────────────────────────────────────────────────────
  void ResolveJointIndices() noexcept;
  void ApplyCommand() noexcept;
  void ReadState() noexcept;
  void InvokeStateCallback() noexcept;
  void UpdateVizBuffer() noexcept;
  void UpdateRtf(uint64_t step) noexcept;
  void ThrottleIfNeeded() noexcept;

  // Reset physics to cfg_.initial_qpos.  Must be called from the sim thread.
  void HandleReset() noexcept;

  void SimLoopFreeRun(std::stop_token stop) noexcept;
  void SimLoopSyncStep(std::stop_token stop) noexcept;
  void ViewerLoop(std::stop_token stop) noexcept;
};

// ── Joint name table (must match UR driver / custom_controller) ───────────────
static constexpr std::array<const char*, 6> kMjJointNames = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
};

// ── Inline implementation ──────────────────────────────────────────────────────

inline MuJoCoSimulator::MuJoCoSimulator(Config cfg) noexcept
    : cfg_(std::move(cfg)) {
  current_max_rtf_.store(cfg_.max_rtf, std::memory_order_relaxed);
}

inline MuJoCoSimulator::~MuJoCoSimulator() {
  Stop();
  if (data_)  { mj_deleteData(data_);   data_  = nullptr; }
  if (model_) { mj_deleteModel(model_); model_ = nullptr; }
}

inline void MuJoCoSimulator::ResolveJointIndices() noexcept {
  for (std::size_t i = 0; i < 6; ++i) {
    const int jnt_id = mj_name2id(model_, mjOBJ_JOINT, kMjJointNames[i]);
    if (jnt_id < 0) {
      fprintf(stderr,
              "[MuJoCoSimulator] Joint '%s' not found in model "
              "— falling back to static index %zu\n",
              kMjJointNames[i], i);
      joint_qpos_indices_[i] = static_cast<int>(i);
      joint_qvel_indices_[i] = static_cast<int>(i);
    } else {
      joint_qpos_indices_[i] = model_->jnt_qposadr[jnt_id];
      joint_qvel_indices_[i] = model_->jnt_dofadr[jnt_id];
      fprintf(stdout,
              "[MuJoCoSimulator] '%s' → qpos[%d]  qvel[%d]\n",
              kMjJointNames[i],
              joint_qpos_indices_[i],
              joint_qvel_indices_[i]);
    }
  }
}

inline bool MuJoCoSimulator::Initialize() noexcept {
  char error[1000] = {};
  model_ = mj_loadXML(cfg_.model_path.c_str(), nullptr, error, sizeof(error));
  if (!model_) {
    fprintf(stderr, "[MuJoCoSimulator] Failed to load '%s': %s\n",
            cfg_.model_path.c_str(), error);
    return false;
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    fprintf(stderr, "[MuJoCoSimulator] mj_makeData failed\n");
    mj_deleteModel(model_);
    model_ = nullptr;
    return false;
  }

  viz_qpos_.assign(static_cast<std::size_t>(model_->nq), 0.0);
  ResolveJointIndices();

  const int njoints = std::min(6, model_->nq);
  for (int i = 0; i < njoints; ++i) {
    const double q0 = cfg_.initial_qpos[static_cast<std::size_t>(i)];
    const int    qi = joint_qpos_indices_[static_cast<std::size_t>(i)];
    data_->qpos[qi] = q0;
    data_->ctrl[i]  = q0;
  }

  mj_forward(model_, data_);
  ReadState();

  fprintf(stdout,
          "[MuJoCoSimulator] Loaded '%s'  nq=%d  nv=%d  nu=%d  dt=%.4f s"
          "  mode=%s\n",
          cfg_.model_path.c_str(),
          model_->nq, model_->nv, model_->nu,
          static_cast<double>(model_->opt.timestep),
          cfg_.mode == SimMode::kFreeRun ? "free_run" : "sync_step");
  return true;
}

inline void MuJoCoSimulator::Start() noexcept {
  if (running_.exchange(true)) { return; }

  if (cfg_.mode == SimMode::kFreeRun) {
    sim_thread_ = std::jthread([this](std::stop_token st) { SimLoopFreeRun(st); });
  } else {
    sim_thread_ = std::jthread([this](std::stop_token st) { SimLoopSyncStep(st); });
  }

  if (cfg_.enable_viewer) {
    viewer_thread_ = std::jthread([this](std::stop_token st) { ViewerLoop(st); });
  }
}

inline void MuJoCoSimulator::Stop() noexcept {
  running_.store(false);
  sync_cv_.notify_all();
  if (sim_thread_.joinable()) {
    sim_thread_.request_stop();
    sim_thread_.join();
  }
  if (viewer_thread_.joinable()) {
    viewer_thread_.request_stop();
    viewer_thread_.join();
  }
}

inline void MuJoCoSimulator::SetCommand(const std::array<double, 6>& cmd) noexcept {
  {
    std::lock_guard lock(cmd_mutex_);
    pending_cmd_ = cmd;
  }
  cmd_pending_.store(true, std::memory_order_release);
  if (cfg_.mode == SimMode::kSyncStep) {
    sync_cv_.notify_one();
  }
}

inline void MuJoCoSimulator::SetStateCallback(StateCallback cb) noexcept {
  state_cb_ = std::move(cb);
}

inline std::array<double, 6> MuJoCoSimulator::GetPositions() const noexcept {
  std::lock_guard lock(state_mutex_);
  return latest_positions_;
}

inline std::array<double, 6> MuJoCoSimulator::GetVelocities() const noexcept {
  std::lock_guard lock(state_mutex_);
  return latest_velocities_;
}

// ── Private helpers ────────────────────────────────────────────────────────────

inline void MuJoCoSimulator::ApplyCommand() noexcept {
  std::lock_guard lock(cmd_mutex_);
  if (!model_) { return; }
  const int nact = std::min(6, model_->nu);
  for (int i = 0; i < nact; ++i) {
    data_->ctrl[i] = pending_cmd_[static_cast<std::size_t>(i)];
  }
}

inline void MuJoCoSimulator::ReadState() noexcept {
  if (!model_ || !data_) { return; }
  std::lock_guard lock(state_mutex_);
  for (std::size_t i = 0; i < 6; ++i) {
    latest_positions_[i]  = data_->qpos[joint_qpos_indices_[i]];
    latest_velocities_[i] = data_->qvel[joint_qvel_indices_[i]];
  }
}

inline void MuJoCoSimulator::InvokeStateCallback() noexcept {
  if (!state_cb_) { return; }
  std::array<double, 6> pos{}, vel{};
  {
    std::lock_guard lock(state_mutex_);
    pos = latest_positions_;
    vel = latest_velocities_;
  }
  state_cb_(pos, vel);
}

inline void MuJoCoSimulator::UpdateVizBuffer() noexcept {
  if (viz_mutex_.try_lock()) {
    std::memcpy(viz_qpos_.data(), data_->qpos,
                static_cast<std::size_t>(model_->nq) * sizeof(double));
    viz_ncon_  = data_->ncon;
    viz_dirty_ = true;
    viz_mutex_.unlock();
  }
}

inline void MuJoCoSimulator::UpdateRtf(uint64_t step) noexcept {
  if (step % 200 != 0) { return; }

  const auto   wall_now = std::chrono::steady_clock::now();
  const double wall_dt  =
      std::chrono::duration<double>(wall_now - rtf_wall_start_).count();
  const double sim_dt   = data_->time - rtf_sim_start_;

  if (wall_dt > 0.01) {
    rtf_.store(sim_dt / wall_dt, std::memory_order_relaxed);
    rtf_wall_start_ = wall_now;
    rtf_sim_start_  = data_->time;
  }
}

// ── ThrottleIfNeeded ───────────────────────────────────────────────────────────
//
// Reads current_max_rtf_ atomically.  When the value changes, the cumulative
// window is reset so the new limit takes effect immediately.
//
inline void MuJoCoSimulator::ThrottleIfNeeded() noexcept {
  const double max_rtf = current_max_rtf_.load(std::memory_order_relaxed);

  // Reset throttle window when the limit changes.
  if (max_rtf != throttle_rtf_) {
    throttle_wall_start_ = std::chrono::steady_clock::now();
    throttle_sim_start_  = data_->time;
    throttle_rtf_        = max_rtf;
  }

  if (max_rtf <= 0.0) { return; }

  const double sim_elapsed  = data_->time - throttle_sim_start_;
  const double target_wall  = sim_elapsed / max_rtf;
  const double actual_wall  = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - throttle_wall_start_).count();

  if (actual_wall < target_wall) {
    std::this_thread::sleep_for(
        std::chrono::duration<double>(target_wall - actual_wall));
  }
}

// ── HandleReset ───────────────────────────────────────────────────────────────
//
// Reinitialises physics to cfg_.initial_qpos.
// Must only be called from the sim thread (sole writer of model_/data_).
//
inline void MuJoCoSimulator::HandleReset() noexcept {
  mj_resetData(model_, data_);

  const int n = std::min(6, model_->nq);
  for (int i = 0; i < n; ++i) {
    const std::size_t ui = static_cast<std::size_t>(i);
    data_->qpos[joint_qpos_indices_[ui]] = cfg_.initial_qpos[ui];
    data_->ctrl[i]                       = cfg_.initial_qpos[ui];
  }
  mj_forward(model_, data_);

  step_count_.store(0,   std::memory_order_relaxed);
  sim_time_sec_.store(0.0, std::memory_order_relaxed);
  rtf_.store(0.0, std::memory_order_relaxed);

  const auto now     = std::chrono::steady_clock::now();
  rtf_wall_start_    = now;  rtf_sim_start_    = data_->time;
  throttle_wall_start_ = now; throttle_sim_start_ = data_->time;
  throttle_rtf_      = current_max_rtf_.load(std::memory_order_relaxed);

  ReadState();
  fprintf(stdout, "[MuJoCoSimulator] Reset to initial pose\n");
}

// ── SimLoopFreeRun ─────────────────────────────────────────────────────────────
//
// Advances physics as fast as possible (throttled by current_max_rtf_).
// Respects pause_ and reset_requested_ flags from any thread.
//
inline void MuJoCoSimulator::SimLoopFreeRun(std::stop_token stop) noexcept {
  if (!model_ || !data_) { return; }

  const auto decim = static_cast<uint64_t>(
      cfg_.publish_decimation > 0 ? cfg_.publish_decimation : 1);

  uint64_t step = 0;

  const auto loop_start = std::chrono::steady_clock::now();
  rtf_wall_start_      = loop_start;  rtf_sim_start_       = data_->time;
  throttle_wall_start_ = loop_start;  throttle_sim_start_  = data_->time;
  throttle_rtf_        = current_max_rtf_.load(std::memory_order_relaxed);

  while (!stop.stop_requested() && running_.load()) {
    // ── Pause: sleep without advancing physics ─────────────────────────────
    if (paused_.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }

    // ── Reset: reinitialise to initial pose ────────────────────────────────
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      HandleReset();
      step = 0;
      continue;
    }

    // ── Apply pending command (lock-free fast path) ────────────────────────
    if (cmd_pending_.load(std::memory_order_acquire)) {
      ApplyCommand();
      cmd_pending_.store(false, std::memory_order_release);
    }

    mj_step(model_, data_);
    ++step;
    step_count_.store(step, std::memory_order_relaxed);
    sim_time_sec_.store(data_->time, std::memory_order_relaxed);

    if (step % decim == 0) {
      ReadState();
      InvokeStateCallback();
    }

    UpdateRtf(step);
    ThrottleIfNeeded();

    if ((step % 8 == 0) && cfg_.enable_viewer) {
      UpdateVizBuffer();
    }
  }

  fprintf(stdout,
          "[MuJoCoSimulator] FreeRun exited — steps=%lu  sim_time=%.3f s\n",
          static_cast<unsigned long>(step_count_.load()),
          sim_time_sec_.load());
}

// ── SimLoopSyncStep ────────────────────────────────────────────────────────────
//
// 1. Publish current state.
// 2. Wait for one command (or timeout / resume / reset).
// 3. Apply command and take one physics step.
//
inline void MuJoCoSimulator::SimLoopSyncStep(std::stop_token stop) noexcept {
  if (!model_ || !data_) { return; }

  const auto timeout = std::chrono::milliseconds(
      static_cast<int64_t>(cfg_.sync_timeout_ms > 0.0 ? cfg_.sync_timeout_ms : 50.0));

  uint64_t step = 0;

  const auto loop_start_ss = std::chrono::steady_clock::now();
  rtf_wall_start_          = loop_start_ss; rtf_sim_start_         = data_->time;
  throttle_wall_start_     = loop_start_ss; throttle_sim_start_    = data_->time;
  throttle_rtf_            = current_max_rtf_.load(std::memory_order_relaxed);

  while (!stop.stop_requested() && running_.load()) {
    // ── Pause ─────────────────────────────────────────────────────────────
    if (paused_.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }

    // ── Reset ─────────────────────────────────────────────────────────────
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      HandleReset();
      step = 0;
      continue;
    }

    // 1. Publish current state.
    ReadState();
    InvokeStateCallback();

    // 2. Wait for command (or timeout / stop / resume / reset).
    {
      std::unique_lock lock(sync_mutex_);
      sync_cv_.wait_for(lock, timeout, [this, &stop] {
        return cmd_pending_.load(std::memory_order_relaxed)
            || stop.stop_requested()
            || !running_.load()
            || reset_requested_.load(std::memory_order_relaxed);
      });
    }

    if (stop.stop_requested() || !running_.load()) { break; }

    // Handle reset signalled during the wait.
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
      HandleReset();
      step = 0;
      continue;
    }

    // 3. Apply command and step.
    if (cmd_pending_.load(std::memory_order_acquire)) {
      ApplyCommand();
      cmd_pending_.store(false, std::memory_order_release);
    }
    mj_step(model_, data_);
    ++step;
    step_count_.store(step, std::memory_order_relaxed);
    sim_time_sec_.store(data_->time, std::memory_order_relaxed);

    UpdateRtf(step);
    ThrottleIfNeeded();

    if ((step % 8 == 0) && cfg_.enable_viewer) {
      UpdateVizBuffer();
    }
  }

  fprintf(stdout,
          "[MuJoCoSimulator] SyncStep exited — steps=%lu  sim_time=%.3f s\n",
          static_cast<unsigned long>(step_count_.load()),
          sim_time_sec_.load());
}

// ── ViewerLoop ─────────────────────────────────────────────────────────────────
inline void MuJoCoSimulator::ViewerLoop(std::stop_token stop) noexcept {
#ifdef MUJOCO_HAVE_GLFW
  if (!glfwInit()) {
    fprintf(stderr, "[MuJoCoSimulator] glfwInit failed — viewer disabled\n");
    return;
  }

  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

  GLFWwindow* window = glfwCreateWindow(
      1280, 960, "UR5e MuJoCo Simulator", nullptr, nullptr);
  if (!window) {
    fprintf(stderr,
            "[MuJoCoSimulator] glfwCreateWindow failed — viewer disabled\n");
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // ── MuJoCo rendering objects ──────────────────────────────────────────────
  mjvCamera  cam;  mjv_defaultCamera(&cam);
  mjvOption  opt;  mjv_defaultOption(&opt);
  mjvScene   scn;  mjv_defaultScene(&scn);
  mjrContext con;  mjr_defaultContext(&con);

  mjv_makeScene(model_, &scn, 2000);
  mjr_makeContext(model_, &con, mjFONTSCALE_150);

  cam.type      = mjCAMERA_FREE;
  cam.distance  = 2.5;
  cam.azimuth   = 90.0;
  cam.elevation = -20.0;

  // ── RTF profiler figure ───────────────────────────────────────────────────
  mjvFigure fig_profiler;
  mjv_defaultFigure(&fig_profiler);
  std::strncpy(fig_profiler.title,       "RTF History",
               sizeof(fig_profiler.title) - 1);
  std::strncpy(fig_profiler.xlabel,      "Frames",
               sizeof(fig_profiler.xlabel) - 1);
  std::strncpy(fig_profiler.linename[0], "RTF",
               sizeof(fig_profiler.linename[0]) - 1);
  // Semi-transparent dark background
  fig_profiler.figurergba[0] = 0.1f;
  fig_profiler.figurergba[1] = 0.1f;
  fig_profiler.figurergba[2] = 0.1f;
  fig_profiler.figurergba[3] = 0.85f;
  // Green line
  fig_profiler.linergb[0][0] = 0.2f;
  fig_profiler.linergb[0][1] = 1.0f;
  fig_profiler.linergb[0][2] = 0.4f;
  fig_profiler.linewidth     = 1.5f;
  // Y-axis: 0–30× RTF; X-axis: 0–200 frames
  fig_profiler.range[0][0] = 0;  fig_profiler.range[0][1] = 200;
  fig_profiler.range[1][0] = 0;  fig_profiler.range[1][1] = 30;
  fig_profiler.flg_extend  = 1;  // auto-extend Y if RTF exceeds 30

  // Viewer interaction state — shared with GLFW callbacks via window user pointer.
  // Defined as a local struct so that captureless lambdas can reach it through
  // glfwGetWindowUserPointer().
  struct ViewerState {
    // MuJoCo rendering handles (non-owning)
    mjvCamera*       cam;
    mjvOption*       opt;
    mjvScene*        scn;
    const mjModel*   model;
    MuJoCoSimulator* sim;

    // Mouse tracking
    bool   btn_left{false};
    bool   btn_right{false};
    double lastx{0.0};
    double lasty{0.0};

    // UI toggles
    bool show_profiler{false};

    // RTF rolling buffer (200 samples at ~60 Hz ≈ 3.3 s window)
    static constexpr int kProfLen = 200;
    float rtf_history[kProfLen]{};
    int   rtf_head{0};
    int   rtf_count{0};  // samples pushed so far, capped at kProfLen

    void push_rtf(float v) noexcept {
      rtf_history[rtf_head] = v;
      rtf_head = (rtf_head + 1) % kProfLen;
      if (rtf_count < kProfLen) { ++rtf_count; }
    }

    // Fill mjvFigure line 0 from the rolling buffer (oldest → newest).
    void update_figure(mjvFigure& fig) const noexcept {
      const int n = rtf_count;
      if (n == 0) { return; }
      // oldest entry is at (rtf_head - n + kProfLen) % kProfLen when full
      const int oldest = (rtf_count < kProfLen) ? 0
                       : (rtf_head);
      for (int i = 0; i < n; ++i) {
        const int idx = (oldest + i) % kProfLen;
        fig.linedata[0][i * 2]     = static_cast<float>(i);
        fig.linedata[0][i * 2 + 1] = rtf_history[idx];
      }
      fig.linepnt[0] = n;
    }
  } vs{&cam, &opt, &scn, model_, this};

  glfwSetWindowUserPointer(window, &vs);

  // ── GLFW callbacks ────────────────────────────────────────────────────────
  //
  // All callbacks are captureless lambdas → implicit function pointer
  // conversion is valid.  State is retrieved via glfwGetWindowUserPointer().

  // Keyboard: Space=pause, +/-=speed, R=reset, F3=profiler,
  //           Backspace=reset opt, Escape=reset camera
  glfwSetKeyCallback(window,
    [](GLFWwindow* w, int key, int /*scan*/, int action, int /*mods*/) noexcept {
      if (action == GLFW_RELEASE) { return; }
      auto* s = static_cast<ViewerState*>(glfwGetWindowUserPointer(w));
      if (!s) { return; }
      switch (key) {
        case GLFW_KEY_SPACE:
          if (s->sim->IsPaused()) { s->sim->Resume(); }
          else                    { s->sim->Pause();  }
          break;

        case GLFW_KEY_EQUAL:       // '=' (same physical key as '+' unshifted)
        case GLFW_KEY_KP_ADD: {
          const double cur = s->sim->GetMaxRtf();
          s->sim->SetMaxRtf(cur <= 0.0 ? 2.0 : cur * 2.0);
          break;
        }

        case GLFW_KEY_MINUS:
        case GLFW_KEY_KP_SUBTRACT: {
          const double cur = s->sim->GetMaxRtf();
          // Below 0.5× → revert to unlimited
          if (cur > 0.0 && cur <= 0.5) { s->sim->SetMaxRtf(0.0); }
          else if (cur > 0.5)          { s->sim->SetMaxRtf(cur / 2.0); }
          break;
        }

        case GLFW_KEY_R:
          s->sim->RequestReset();
          break;

        case GLFW_KEY_F3:
          s->show_profiler = !s->show_profiler;
          break;

        case GLFW_KEY_BACKSPACE:
          mjv_defaultOption(s->opt);
          break;

        case GLFW_KEY_ESCAPE:
          mjv_defaultCamera(s->cam);
          s->cam->distance  = 2.5;
          s->cam->azimuth   = 90.0;
          s->cam->elevation = -20.0;
          break;

        default: break;
      }
    });

  // Mouse button: track which buttons are pressed + cursor anchor
  glfwSetMouseButtonCallback(window,
    [](GLFWwindow* w, int button, int action, int /*mods*/) noexcept {
      auto* s = static_cast<ViewerState*>(glfwGetWindowUserPointer(w));
      if (!s) { return; }
      const bool pressed = (action == GLFW_PRESS);
      if (button == GLFW_MOUSE_BUTTON_LEFT)  { s->btn_left  = pressed; }
      if (button == GLFW_MOUSE_BUTTON_RIGHT) { s->btn_right = pressed; }
      // Anchor the drag start position on press
      if (pressed) {
        glfwGetCursorPos(w, &s->lastx, &s->lasty);
      }
    });

  // Cursor movement: orbit (left drag) or pan (right drag)
  glfwSetCursorPosCallback(window,
    [](GLFWwindow* w, double xpos, double ypos) noexcept {
      auto* s = static_cast<ViewerState*>(glfwGetWindowUserPointer(w));
      if (!s || (!s->btn_left && !s->btn_right)) {
        // No button held — just update position
        s->lastx = xpos;
        s->lasty = ypos;
        return;
      }

      int width = 0, height = 0;
      glfwGetWindowSize(w, &width, &height);
      if (width == 0 || height == 0) { return; }

      const double dx = xpos - s->lastx;
      const double dy = ypos - s->lasty;
      s->lastx = xpos;
      s->lasty = ypos;

      const double nx = dx / static_cast<double>(width);
      const double ny = dy / static_cast<double>(height);

      if (s->btn_left) {
        // Left drag → orbit (separate H and V passes for smooth rotation)
        mjv_moveCamera(s->model, mjMOUSE_ROTATE_H, nx, 0.0, s->scn, s->cam);
        mjv_moveCamera(s->model, mjMOUSE_ROTATE_V, 0.0, ny, s->scn, s->cam);
      } else if (s->btn_right) {
        // Right drag → pan (translate camera target)
        mjv_moveCamera(s->model, mjMOUSE_MOVE_H, nx, 0.0, s->scn, s->cam);
        mjv_moveCamera(s->model, mjMOUSE_MOVE_V, 0.0, ny, s->scn, s->cam);
      }
    });

  // Scroll: zoom
  glfwSetScrollCallback(window,
    [](GLFWwindow* w, double /*xoffset*/, double yoffset) noexcept {
      auto* s = static_cast<ViewerState*>(glfwGetWindowUserPointer(w));
      if (!s) { return; }
      mjv_moveCamera(s->model, mjMOUSE_ZOOM, 0.0,
                     -0.05 * yoffset, s->scn, s->cam);
    });

  // Visualization-only mjData — never written by the sim thread.
  mjData* vis_data = mj_makeData(model_);
  {
    std::lock_guard lock(state_mutex_);
    for (std::size_t i = 0; i < 6; ++i) {
      vis_data->qpos[joint_qpos_indices_[i]] = latest_positions_[i];
    }
  }
  mj_forward(model_, vis_data);

  fprintf(stdout,
          "[MuJoCoSimulator] Viewer running\n"
          "  Space:pause  +/-:speed  R:reset  F3:profiler  Esc:camera\n"
          "  Left-drag:orbit  Right-drag:pan  Scroll:zoom\n");

  // ── Render loop ────────────────────────────────────────────────────────────
  while (!stop.stop_requested() && running_.load() &&
         !glfwWindowShouldClose(window)) {

    // ── Sync viz buffer from sim thread ─────────────────────────────────────
    int ncon_snap = 0;
    {
      std::lock_guard lock(viz_mutex_);
      if (viz_dirty_) {
        std::memcpy(vis_data->qpos, viz_qpos_.data(),
                    static_cast<std::size_t>(model_->nq) * sizeof(double));
        viz_dirty_ = false;
      }
      ncon_snap = viz_ncon_;
    }
    mj_forward(model_, vis_data);

    // ── Sample RTF into rolling buffer (60 Hz) ───────────────────────────────
    const float cur_rtf = static_cast<float>(rtf_.load(std::memory_order_relaxed));
    vs.push_rtf(cur_rtf);
    vs.update_figure(fig_profiler);

    // ── Render scene ──────────────────────────────────────────────────────────
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    mjrRect viewport{0, 0, width, height};

    mjv_updateScene(model_, vis_data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // ── Status overlay (top-right) ────────────────────────────────────────────
    {
      const double   max_rtf_val = current_max_rtf_.load(std::memory_order_relaxed);
      const bool     is_paused   = paused_.load(std::memory_order_relaxed);
      const SimMode  mode        = cfg_.mode;

      char limit_str[32];
      if (max_rtf_val > 0.0) {
        std::snprintf(limit_str, sizeof(limit_str), "%.1fx", max_rtf_val);
      } else {
        std::strncpy(limit_str, "unlimited", sizeof(limit_str) - 1);
        limit_str[sizeof(limit_str) - 1] = '\0';
      }

      char labels[256], values[256];
      std::snprintf(labels, sizeof(labels),
                    "Mode\nRTF\nLimit\nSim Time\nSteps\nContacts\nStatus");
      std::snprintf(values, sizeof(values),
                    "%s\n%.1fx\n%s\n%.2f s\n%lu\n%d\n%s",
                    mode == SimMode::kFreeRun ? "free_run" : "sync_step",
                    static_cast<double>(cur_rtf),
                    limit_str,
                    sim_time_sec_.load(std::memory_order_relaxed),
                    static_cast<unsigned long>(step_count_.load(std::memory_order_relaxed)),
                    ncon_snap,
                    is_paused ? "PAUSED" : "running");
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport,
                  labels, values, &con);
    }

    // ── Help overlay (bottom-left) ────────────────────────────────────────────
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                "Space:pause  +/-:speed  R:reset  F3:profiler\n"
                "Left-drag:orbit  Right-drag:pan  Scroll:zoom  Esc:camera",
                nullptr, &con);

    // ── Profiler figure (bottom-right, toggled by F3) ─────────────────────────
    if (vs.show_profiler && vs.rtf_count > 0) {
      const int fw = width  / 3;
      const int fh = height / 3;
      mjrRect fig_rect{width - fw, 0, fw, fh};
      mjr_figure(fig_rect, &fig_profiler, &con);
    }

    glfwSwapBuffers(window);
    glfwPollEvents();

    std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 Hz
  }

  mj_deleteData(vis_data);
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  glfwDestroyWindow(window);
  glfwTerminate();
  fprintf(stdout, "[MuJoCoSimulator] Viewer closed\n");

#else
  fprintf(stdout,
          "[MuJoCoSimulator] Viewer not available "
          "(build without -DMUJOCO_HAVE_GLFW)\n");
  (void)stop;
#endif
}

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_
