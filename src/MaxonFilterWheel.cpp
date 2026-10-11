#ifndef MOCK_WHEEL
#include "MaxonFilterWheel.hpp"

#include <Definitions.h>
#include <chrono>
#include <climits>
#include <cstdio>
#include <thread>

// EPOS Command Library uses C-style TRUE/FALSE macros.
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

using namespace std::chrono_literals;

namespace andor2k {

// Profile position parameters (tune after hardware testing)
static constexpr unsigned int PROFILE_VELOCITY = 6000;      // rpm
static constexpr unsigned int PROFILE_ACCELERATION = 10000; // rpm/s
static constexpr unsigned int PROFILE_DECELERATION = 10000; // rpm/s

// Decode an EposCmd error code to a human-readable string and print it.
// VCS_GetErrorInfo(code, buf, maxLen) fills buf with the description.
static void log_epos_error(const char *fn, unsigned short node,
                           unsigned int err_code) {
  char info[256] = "(no description)";
  VCS_GetErrorInfo(err_code, info, sizeof(info));
  printf("[MaxonFilterWheel] %s failed (node %u  err 0x%08X: %s)\n", fn, node,
         err_code, info);
}

MaxonFilterWheel::MaxonFilterWheel() = default;

MaxonFilterWheel::~MaxonFilterWheel() {
  join_worker();
  Disconnect();
}

void MaxonFilterWheel::join_worker() {
  if (m_worker.joinable())
    m_worker.join();
}

// Open one EPOS4 device on the given USB port.
// Returns handle on success, nullptr on failure.
static void *try_open_epos4(const char *port, unsigned int *err_out) {
  // VCS_OpenDevice takes non-const char*; use local buffers to avoid
  // string-literal-to-char* which is a hard error in C++11+.
  char dev[] = "EPOS4";
  char proto[] = "MAXON SERIAL V2";
  char iface[] = "USB";
  char port_buf[64];
  std::snprintf(port_buf, sizeof(port_buf), "%s", port);
  void *handle = VCS_OpenDevice(dev, proto, iface, port_buf, err_out);
  if (!handle)
    return nullptr;
  // Get current protocol settings first (like HelloEposCmd does) and only
  // override the baudrate, keeping the device's default timeout.
  // Our old hardcoded 500 ms was too short and caused probe_node() to time
  // out on the second USB port after one failed probe on the first.
  unsigned int cur_baud = 0, cur_timeout = 0;
  if (!VCS_GetProtocolStackSettings(handle, &cur_baud, &cur_timeout, err_out)) {
    VCS_CloseDevice(handle, err_out);
    return nullptr;
  }
  if (!VCS_SetProtocolStackSettings(handle, 1000000, cur_timeout, err_out)) {
    VCS_CloseDevice(handle, err_out);
    return nullptr;
  }
  return handle;
}

// Probe whether a device handle responds to the given node ID.
// Uses VCS_GetFaultState - a lightweight read that works in any drive state.
static bool probe_node(void *handle, unsigned short node_id) {
  unsigned int err = 0;
  int fault = 0;
  return VCS_GetFaultState(handle, node_id, &fault, &err) != 0;
}

int MaxonFilterWheel::init_node(void *handle, unsigned short node_id) {
  unsigned int err = 0;

  // Clear any existing faults
  int fault = 0;
  VCS_GetFaultState(handle, node_id, &fault, &err);
  if (fault)
    VCS_ClearFault(handle, node_id, &err);

  if (!VCS_SetEnableState(handle, node_id, &err)) {
    printf("[MaxonFilterWheel] SetEnableState failed (node %u, err %u)\n",
           node_id, err);
    return 1;
  }

  if (!VCS_ActivateProfilePositionMode(handle, node_id, &err)) {
    printf("[MaxonFilterWheel] ActivateProfilePositionMode failed "
           "(node %u, err %u)\n",
           node_id, err);
    return 1;
  }

  if (!VCS_SetPositionProfile(handle, node_id, PROFILE_VELOCITY,
                              PROFILE_ACCELERATION, PROFILE_DECELERATION,
                              &err)) {
    printf("[MaxonFilterWheel] SetPositionProfile failed (node %u, err %u)\n",
           node_id, err);
    return 1;
  }

  return 0;
}

int MaxonFilterWheel::Connect() {
  // Scan USB0..USB3 and identify each drive by its node ID, NOT by port order.
  // The upper drive must be configured with NODE_UPPER (1) in EPOS Studio;
  // the lower with NODE_LOWER (2). Non-EPOS USB devices are silently skipped
  // because probe_node() will fail for them.
  unsigned int err = 0;
  const char *ports[] = {"USB0", "USB1", "USB2", "USB3"};

  for (const char *port : ports) {
    if (m_handle_upper && m_handle_lower)
      break;

    void *h = try_open_epos4(port, &err);
    if (!h)
      continue;

    if (!m_handle_upper && probe_node(h, NODE_UPPER)) {
      m_handle_upper = h;
      printf("[MaxonFilterWheel] Upper wheel (node %u) on %s\n", NODE_UPPER,
             port);
    } else if (!m_handle_lower && probe_node(h, NODE_LOWER)) {
      m_handle_lower = h;
      printf("[MaxonFilterWheel] Lower wheel (node %u) on %s\n", NODE_LOWER,
             port);
    } else {
      // Neither expected node responded. Scan nodes 1-14 to show which
      // node ID the device actually has (helps diagnose DIP-switch issues).
      printf("[MaxonFilterWheel] Device on %s did not answer node %u or %u - "
             "scanning for its actual node ID...\n",
             port, NODE_UPPER, NODE_LOWER);
      bool found_any = false;
      for (unsigned short n = 1; n <= 14; ++n) {
        if (probe_node(h, n)) {
          printf("[MaxonFilterWheel]   -> %s answers to node %u "
                 "(expected %u or %u)\n",
                 port, n, NODE_UPPER, NODE_LOWER);
          found_any = true;
        }
      }
      if (!found_any)
        printf("[MaxonFilterWheel]   -> no node 1-14 responded on %s "
               "(drive may be off or in error)\n",
               port);
      VCS_CloseDevice(h, &err);
    }
  }

  if (!m_handle_upper) {
    printf("[MaxonFilterWheel] WARNING: Upper wheel (node %u) not found; "
           "GoHome/SetPosition will be unavailable\n",
           NODE_UPPER);
  }
  if (!m_handle_lower) {
    printf("[MaxonFilterWheel] WARNING: Lower wheel (node %u) not found; "
           "GoHome/SetPosition will be unavailable\n",
           NODE_LOWER);
  }
  if (!m_handle_upper && !m_handle_lower) {
    printf("[MaxonFilterWheel] ERROR: No drives found\n");
    return 1;
  }

  // Guard: only init wheels that were actually found. Calling init_node()
  // with a null handle causes VCS_* to crash with an invalid-handle error.
  if (m_handle_upper && init_node(m_handle_upper, NODE_UPPER) != 0) {
    Disconnect();
    return 2;
  }
  if (m_handle_lower && init_node(m_handle_lower, NODE_LOWER) != 0) {
    Disconnect();
    return 3;
  }

  printf("[MaxonFilterWheel] Connected; both nodes initialised\n");
  return 0;
}

int MaxonFilterWheel::Disconnect() {
  // Join the worker first so we never close EPOS4 handles while a VCS_* call
  // is still in flight (e.g. when called from kill_daemon after Halt()).
  join_worker();
  unsigned int err = 0;
  if (m_handle_upper) {
    VCS_SetDisableState(m_handle_upper, NODE_UPPER, &err);
    VCS_CloseDevice(m_handle_upper, &err);
    m_handle_upper = nullptr;
  }
  if (m_handle_lower) {
    VCS_SetDisableState(m_handle_lower, NODE_LOWER, &err);
    VCS_CloseDevice(m_handle_lower, &err);
    m_handle_lower = nullptr;
  }
  printf("[MaxonFilterWheel] Disconnected\n");
  return 0;
}

void MaxonFilterWheel::update_qc(std::atomic<long> &acc, int &prev_raw,
                                 void *handle, unsigned short node_id) {
  unsigned int err = 0;
  int raw = 0;
  if (!VCS_GetPositionIs(handle, node_id, &raw, &err))
    return;

  // Detect 32-bit wraparound and update 64-bit accumulator
  int delta = raw - prev_raw;
  // If delta is suspiciously large (> half INT range) it wrapped
  if (delta < INT_MIN / 2)
    delta += (int)(1LL << 32);
  else if (delta > INT_MAX / 2)
    delta -= (int)(1LL << 32);

  acc.fetch_add((long)delta);
  prev_raw = raw;
}

// Index-pulse sensor state machine.
// Called each poll cycle in SetPosition() for each wheel.
// Reads DI1 (object 0x60FD bit 16), detects edges, and on the FIRST rising
// edge of each double-pulse pair snaps the QC accumulator to the nearest
// rotation boundary to eliminate belt/pulley drift.
// The counter is NOT reset to zero - it is corrected in-place by removing
// drift only (snap amount is always << QC_PER_SLOT).
void MaxonFilterWheel::poll_sensor(void *handle, unsigned short node_id,
                                   std::atomic<long> &qc, SensorState &state,
                                   bool &di1_prev, long sensor_offset) {
  // DI1 = bit 16 of object 0x60FD (EPOS4 digital inputs, manufacturer-specific)
  unsigned int di_val = 0, bytes = 0, err = 0;
  VCS_GetObject(handle, node_id, 0x60FD, 0x00, &di_val, sizeof(di_val), &bytes,
                &err);
  const bool di1 = (di_val >> 16) & 1u;
  const bool rising = di1 && !di1_prev;
  const bool falling = !di1 && di1_prev;
  di1_prev = di1;

  switch (state) {
  case SensorState::IDLE:
    if (rising) {
      long current = qc.load();
      // Sensor fires at sensor_offset + N * QC_PER_ROTATION.
      // Snap to the nearest such boundary to remove accumulated drift.
      long n = (current - sensor_offset) / DualWheelMapper::QC_PER_ROTATION;
      long expected = n * DualWheelMapper::QC_PER_ROTATION + sensor_offset;
      long drift = current - expected;
      long abs_drift = drift < 0 ? -drift : drift;
      printf("[MaxonFilterWheel] DI1\u2191 node %u: QC %ld \u2192 %ld (drift "
             "%+ld QC)\n",
             node_id, current, expected, drift);
      if (abs_drift < DualWheelMapper::MAX_DRIFT_QC) {
        qc.store(expected);
      } else {
        printf("[MaxonFilterWheel] WARNING node %u: drift %ld QC exceeds "
               "threshold - home invalidated\n",
               node_id, abs_drift);
        m_homed.store(false);
      }
      state = SensorState::FIRST_PULSE;
    }
    break;
  case SensorState::FIRST_PULSE:
    if (falling) {
      printf("[MaxonFilterWheel] DI1\u2193 node %u: first pulse ended\n",
             node_id);
      state = SensorState::GAP;
    }
    break;
  case SensorState::GAP:
    if (rising) {
      printf("[MaxonFilterWheel] DI1\u2191 node %u: second pulse (ignored)\n",
             node_id);
      state = SensorState::SECOND_PULSE;
    }
    break;
  case SensorState::SECOND_PULSE:
    if (falling) {
      printf(
          "[MaxonFilterWheel] DI1\u2193 node %u: second pulse ended - armed\n",
          node_id);
      state = SensorState::IDLE;
    }
    break;
  }
}

int MaxonFilterWheel::GoHome() {
  if (!m_handle_upper || !m_handle_lower)
    return 1;
  if (m_moving.load())
    return 2;

  join_worker();
  m_halt_requested.store(false);
  m_moving.store(true);
  m_homed.store(false);
  m_pos.store(-1);

  m_worker = std::thread([this]() {
    bool upper_ok = false, lower_ok = false;

    // Run upper on this (worker) thread, lower on a sub-thread.
    // This way: if upper fails early, we can set m_halt_requested BEFORE
    // joining the lower thread, so its DI1 poll loop exits immediately rather
    // than spinning until the sensor fires or the 60-second daemon timeout.
    std::thread lower_thread([&]() {
      lower_ok =
          home_one_wheel(m_handle_lower, NODE_LOWER, m_lower_qc,
                         m_lower_raw_prev, LOWER_HOMING_SENSOR_OFFSET_QC);
    });
    upper_ok = home_one_wheel(m_handle_upper, NODE_UPPER, m_upper_qc,
                              m_upper_raw_prev, UPPER_HOMING_SENSOR_OFFSET_QC);

    // If upper failed (and it wasn't a user halt), cancel the lower thread.
    bool auto_halted = false;
    if (!upper_ok && !m_halt_requested.load()) {
      m_halt_requested.store(true);
      auto_halted = true;
      printf("[MaxonFilterWheel] Upper homing failed - cancelling lower.\n");
    }
    lower_thread.join();
    if (auto_halted)
      m_halt_requested.store(false); // restore for next command

    if (upper_ok && lower_ok && !m_halt_requested.load()) {
      // Re-arm sensor state machines for the first correction after homing.
      m_upper_sensor_state = SensorState::IDLE;
      m_lower_sensor_state = SensorState::IDLE;
      m_upper_di1_prev = false;
      m_lower_di1_prev = false;
      m_homed.store(true);
      // m_pos stays -1 (unknown) - operator should press Go -> slot 0
      // to verify physical alignment after calibrating HOMING_SENSOR_OFFSET_QC.
      printf("[MaxonFilterWheel] Software homing complete.\n"
             "  Upper QC=%ld  Lower QC=%ld\n"
             "  Send 'wheel setpos 0' (clear) to verify alignment.\n",
             m_upper_qc.load(), m_lower_qc.load());
    } else {
      if (m_halt_requested.load())
        printf("[MaxonFilterWheel] Homing halted by user.\n");
      else
        printf("[MaxonFilterWheel] Homing FAILED (upper=%d lower=%d).\n",
               upper_ok, lower_ok);
      m_halt_requested.store(false);
    }

    m_moving.store(false);
  });

  return 0;
}

bool MaxonFilterWheel::home_one_wheel(void *handle, unsigned short node,
                                      std::atomic<long> &qc_ref,
                                      int &raw_prev_ref,
                                      long sensor_offset) noexcept {
  unsigned int err = 0;

  // 0. Ensure drive is fault-free and enabled before mode switch
  // ActivateProfileVelocityMode fails if the drive is in a fault state or
  // disabled (e.g. after a previous halt or error).
  int fault = 0;
  VCS_GetFaultState(handle, node, &fault, &err);
  if (fault) {
    printf("[MaxonFilterWheel] node %u: clearing fault before homing\n", node);
    if (!VCS_ClearFault(handle, node, &err)) {
      log_epos_error("VCS_ClearFault", node, err);
      return false;
    }
  }
  int enabled = 0;
  VCS_GetEnableState(handle, node, &enabled, &err);
  if (!enabled) {
    if (!VCS_SetEnableState(handle, node, &err)) {
      log_epos_error("VCS_SetEnableState", node, err);
      return false;
    }
  }

  // 1. Switch to profile velocity mode and start forward motion
  if (!VCS_ActivateProfileVelocityMode(handle, node, &err)) {
    log_epos_error("VCS_ActivateProfileVelocityMode", node, err);
    return false;
  }
  if (!VCS_SetVelocityProfile(handle, node, PROFILE_ACCELERATION,
                              PROFILE_DECELERATION, &err)) {
    log_epos_error("VCS_SetVelocityProfile", node, err);
    return false;
  }
  if (!VCS_MoveWithVelocity(handle, node, (long)HOMING_VELOCITY_RPM, &err)) {
    log_epos_error("VCS_MoveWithVelocity", node, err);
    return false;
  }
  printf("[MaxonFilterWheel] node %u: searching for home sensor "
         "at %d RPM...\n",
         node, HOMING_VELOCITY_RPM);

  // 2. Read initial DI1 state, then poll until it changes or timeout
  // Do NOT hardcode initial state: if the sensor is active-low (high at rest),
  // a hardcoded false would give a spurious "rising edge" on the first poll,
  // or detect the TRAILING edge of an active-low pulse instead of the leading
  // one, causing the wheel to overshoot. Detecting ANY change from the real
  // initial state is polarity-agnostic and stops at the first sensor event.
  unsigned int di_val_init = 0;
  {
    unsigned int bytes_i = 0;
    if (!VCS_GetObject(handle, node, 0x60FD, 0x00, &di_val_init,
                       sizeof(di_val_init), &bytes_i, &err))
      log_epos_error("VCS_GetObject(0x60FD) initial", node, err);
  }
  const bool di1_initial = (di_val_init >> 2) & 1u;
  printf("[MaxonFilterWheel] node %u: home_sw initial = %u  "
         "(0x60FD raw = 0x%08X)\n",
         node, (unsigned)di1_initial, di_val_init);

  // Worst-case: wheel at 180 deg from sensor, one full rotation at 3000 RPM
  // motor ~86 s - use 120 s for margin.
  static constexpr int HOMING_TIMEOUT_MS = 120000;
  bool found = false;
  int poll_cnt = 0;
  auto t_start = std::chrono::steady_clock::now();
  while (!m_halt_requested.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    unsigned int di_val = 0, bytes = 0;
    if (!VCS_GetObject(handle, node, 0x60FD, 0x00, &di_val, sizeof(di_val),
                       &bytes, &err))
      log_epos_error("VCS_GetObject(0x60FD)", node, err);
    // Bit 2 of 0x60FD is the CiA 402 home-switch logical input.
    // EPOS4 routes DI1 to this function; the raw DI1 physical bit (16)
    // stays 0 even when the sensor fires.
    const bool home_sw = (di_val >> 2) & 1u;

    // Debug: print DI state every ~500 ms
    if (++poll_cnt % 100 == 0)
      printf("[MaxonFilterWheel] node %u: poll[%d] home_sw=%u  0x60FD=0x%08X\n",
             node, poll_cnt, (unsigned)home_sw, di_val);

    // Any change from the initial state = sensor event (polarity-agnostic)
    if (home_sw != di1_initial) {
      found = true;
      break;
    }

    // Safety timeout
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - t_start)
                          .count();
    if (elapsed_ms > HOMING_TIMEOUT_MS) {
      printf("[MaxonFilterWheel] node %u: homing TIMEOUT (%d ms) - "
             "sensor never triggered\n",
             node, HOMING_TIMEOUT_MS);
      break;
    }
  }

  // 3. Stop motor
  VCS_HaltVelocityMovement(handle, node, &err);
  std::this_thread::sleep_for(50ms); // let motor coast to rest

  auto restore_position_mode = [&]() {
    VCS_ActivateProfilePositionMode(handle, node, &err);
    VCS_SetPositionProfile(handle, node, PROFILE_VELOCITY, PROFILE_ACCELERATION,
                           PROFILE_DECELERATION, &err);
  };

  if (!found || m_halt_requested.load()) {
    printf("[MaxonFilterWheel] node %u: homing aborted.\n", node);
    restore_position_mode();
    return false;
  }

  // 4. Zero the software accumulator at the sensor position
  // VCS_FindHome(HM_ACTUAL_POSITION) was supposed to reset the drive's
  // internal position register, but in practice it silently fails (mode
  // switch error or timing) and VCS_GetPositionIs returns the large raw
  // encoder count rather than 0. We don't need the drive to reset: all
  // slot targets are computed as relative moves against m_upper_qc /
  // m_lower_qc, so we can just set the accumulator to sensor_offset
  // directly. This makes the sensor position the definition of QC = 0
  // (+ any calibrated offset), independent of the drive's internal counter.
  restore_position_mode();

  // Read the actual stopped drive position so that update_qc's first delta
  // is 0 (not a spurious 24M-QC jump caused by raw_prev_ref=0).
  int raw_now = 0;
  unsigned int err_rb = 0;
  VCS_GetPositionIs(handle, node, &raw_now, &err_rb);

  qc_ref.store(sensor_offset); // sensor position IS slot 0 (offset = 0)
  raw_prev_ref = raw_now; // delta baseline matches the drive's current counter

  printf("[MaxonFilterWheel] node %u: homed at sensor - "
         "accumulator = %ld QC  drive raw = %d\n",
         node, sensor_offset, raw_now);
  return true;
}

int MaxonFilterWheel::SetPosition(int virtual_slot) {
  if (virtual_slot < 0 || virtual_slot > 10)
    return 1;
  if (!m_homed.load())
    return 2;
  if (m_moving.load())
    return 3;
  if (!m_handle_upper)
    return 4;

  join_worker();
  m_halt_requested.store(false);
  m_moving.store(true);
  m_pos.store(-1);

  m_worker = std::thread([this, virtual_slot]() {
    unsigned int err = 0;

    DualWheelMapper::PhysicalTarget target =
        DualWheelMapper::MapVirtualToPhysical(virtual_slot, m_upper_qc.load(),
                                              m_lower_qc.load());

    printf(
        "[MaxonFilterWheel] Moving to slot %d (upper_qc=%ld, lower_qc=%ld)\n",
        virtual_slot, target.upper_qc, target.lower_qc);

    // Use RELATIVE moves so the delta sent to the drive is always < 1 rotation
    // (~8.6M QC), well within the 32-bit drive register. Absolute targets
    // accumulate past INT_MAX after ~249 rotations and would be silently
    // truncated by the drive firmware.
    const long delta_upper = target.upper_qc - m_upper_qc.load();
    const long delta_lower = target.lower_qc - m_lower_qc.load();
    printf("[MaxonFilterWheel] Relative deltas: upper=%ld lower=%ld QC\n",
           delta_upper, delta_lower);

    VCS_MoveToPosition(m_handle_upper, NODE_UPPER, delta_upper, FALSE, TRUE,
                       &err);
    VCS_MoveToPosition(m_handle_lower, NODE_LOWER, delta_lower, FALSE, TRUE,
                       &err);

    // Poll until both wheels reach target or halt
    while (!m_halt_requested.load()) {
      update_qc(m_upper_qc, m_upper_raw_prev, m_handle_upper, NODE_UPPER);
      update_qc(m_lower_qc, m_lower_raw_prev, m_handle_lower, NODE_LOWER);

      // DI1 sensor: detect index pulse, correct accumulated drift
      poll_sensor(m_handle_upper, NODE_UPPER, m_upper_qc, m_upper_sensor_state,
                  m_upper_di1_prev, UPPER_HOMING_SENSOR_OFFSET_QC);
      poll_sensor(m_handle_lower, NODE_LOWER, m_lower_qc, m_lower_sensor_state,
                  m_lower_di1_prev, LOWER_HOMING_SENSOR_OFFSET_QC);

      int reached_upper = 0, reached_lower = 0;
      VCS_GetMovementState(m_handle_upper, NODE_UPPER, &reached_upper, &err);
      VCS_GetMovementState(m_handle_lower, NODE_LOWER, &reached_lower, &err);

      if (reached_upper && reached_lower)
        break;

      std::this_thread::sleep_for(50ms);
    }

    if (!m_halt_requested.load()) {
      m_pos.store(virtual_slot);
      printf("[MaxonFilterWheel] Reached slot %d\n", virtual_slot);
    } else {
      printf("[MaxonFilterWheel] Move halted\n");
    }

    m_moving.store(false);
  });

  return 0;
}

int MaxonFilterWheel::Halt() {
  m_halt_requested.store(true);
  if (m_handle_upper) {
    unsigned int err = 0;
    // Stop both position-mode and velocity-mode movements so Halt() works
    // whether called during SetPosition (profile position) or GoHome
    // (velocity).
    VCS_HaltPositionMovement(m_handle_upper, NODE_UPPER, &err);
    VCS_HaltPositionMovement(m_handle_lower, NODE_LOWER, &err);
    VCS_HaltVelocityMovement(m_handle_upper, NODE_UPPER, &err);
    VCS_HaltVelocityMovement(m_handle_lower, NODE_LOWER, &err);
  }
  printf("[MaxonFilterWheel] Halt issued\n");
  return 0;
}

int MaxonFilterWheel::GetPosition() const {
  if (m_moving.load())
    return -1;
  return m_pos.load();
}

bool MaxonFilterWheel::IsMoving() const {
  return m_moving.load();
}

bool MaxonFilterWheel::IsHomed() const {
  return m_homed.load();
}

long MaxonFilterWheel::GetUpperQC() const {
  return m_upper_qc.load();
}

long MaxonFilterWheel::GetLowerQC() const {
  return m_lower_qc.load();
}

std::vector<std::string> MaxonFilterWheel::GetNames() const {
  return {"Clear",    "Filter 1", "Filter 2", "Filter 3",
          "Filter 4", "Filter 5", "Filter 6", "Filter 7",
          "Filter 8", "Filter 9", "Filter 10"};
}

} // namespace andor2k

#endif // MOCK_WHEEL
