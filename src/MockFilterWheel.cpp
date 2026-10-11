#include "MockFilterWheel.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

using namespace std::chrono_literals;

namespace andor2k {

// Step size per simulation tick (QC units). At 5 ms/tick this gives
// ~286 ticks for one full slot rotation, which feels realistic.
static constexpr long SIM_QC_STEP = 5000L;

MockFilterWheel::MockFilterWheel() = default;

MockFilterWheel::~MockFilterWheel() {
  join_worker();
}

void MockFilterWheel::join_worker() {
  if (m_worker.joinable())
    m_worker.join();
}

int MockFilterWheel::Connect() {
  printf("[SIM][MockFilterWheel] Connect() - no-op\n");
  return 0;
}

int MockFilterWheel::Disconnect() {
  join_worker();
  printf("[SIM][MockFilterWheel] Disconnect() - no-op\n");
  return 0;
}

bool MockFilterWheel::simulate_travel(std::atomic<long> &qc,
                                      SensorState &sensor_state, long target_qc,
                                      long sensor_offset,
                                      unsigned short node_id, int step_us) {
  while (qc.load() < target_qc) {
    if (m_halt_requested.load())
      return false;
    long prev_qc = qc.load();
    long next_qc = std::min(prev_qc + SIM_QC_STEP, target_qc);
    qc.store(next_qc);
    check_sensor(qc, sensor_state, prev_qc, next_qc, sensor_offset, node_id);
    std::this_thread::sleep_for(std::chrono::microseconds(step_us));
  }
  return true;
}

// Sensor state machine - called after every QC step.
// Detects rotation boundary crossings (first trigger) and snaps the
// accumulated QC counter to the nearest rotation boundary to eliminate
// belt/pulley drift. The second trigger of each double-pulse pair is
// tracked and silently discarded.
void MockFilterWheel::check_sensor(std::atomic<long> &qc, SensorState &state,
                                   long prev_qc, long next_qc,
                                   long sensor_offset, unsigned short node_id) {
  switch (state) {
  case SensorState::IDLE: {
    long prev_cycle =
        (prev_qc - sensor_offset) / DualWheelMapper::s_qc_per_rotation;
    long next_cycle =
        (next_qc - sensor_offset) / DualWheelMapper::s_qc_per_rotation;
    if (next_cycle > prev_cycle) {
      // Crossed a sensor trigger point - snap accumulator to remove drift.
      long expected_qc =
          next_cycle * DualWheelMapper::s_qc_per_rotation + sensor_offset;
      long current = qc.load();
      long drift = current - expected_qc; // always >= 0 (forward-only)
      printf("[SIM][MockFilterWheel] DI1\xe2\x86\x91 node %u: QC %ld "
             "\xe2\x86\x92 %ld (drift %+ld QC)\n",
             node_id, current, expected_qc, drift);
      if (drift < DualWheelMapper::s_max_drift_qc) {
        qc.store(expected_qc);
      } else {
        printf("[SIM][MockFilterWheel] WARNING node %u: drift %ld QC exceeds "
               "threshold - home invalidated\n",
               node_id, drift);
        m_homed.store(false);
      }
      state = SensorState::FIRST_PULSE;
    }
    break;
  }
  case SensorState::FIRST_PULSE: {
    long pos = (next_qc - sensor_offset) % DualWheelMapper::s_qc_per_rotation;
    if (pos >= SIM_SENSOR_PULSE1_QC) {
      state = SensorState::GAP;
    }
    break;
  }
  case SensorState::GAP: {
    long pos = (next_qc - sensor_offset) % DualWheelMapper::s_qc_per_rotation;
    if (pos >= SIM_SENSOR_PULSE1_QC + SIM_SENSOR_GAP_QC) {
      state = SensorState::SECOND_PULSE;
    }
    break;
  }
  case SensorState::SECOND_PULSE: {
    long pos = (next_qc - sensor_offset) % DualWheelMapper::s_qc_per_rotation;
    if (pos >=
        SIM_SENSOR_PULSE1_QC + SIM_SENSOR_GAP_QC + SIM_SENSOR_PULSE2_QC) {
      state = SensorState::IDLE;
    }
    break;
  }
  }
}

int MockFilterWheel::GoHome() {
  if (m_moving.load())
    return 1;

  join_worker();
  m_halt_requested.store(false);
  m_moving.store(true);
  m_pos.store(-1);

  m_worker = std::thread([this]() {
    printf("[SIM][MockFilterWheel] Homing started\n");
    // Simulate travel: wheels return to QC that is the next multiple of
    // QC_PER_ROTATION >= current. For sim we simply advance to the next
    // rotation boundary from 0 (i.e. both wheels drive to their own next
    // home position). We approximate this as a 2-second delay.
    std::this_thread::sleep_for(2000ms);

    if (!m_halt_requested.load()) {
      // Set accumulators to the per-wheel sensor offset (mirrors real homing).
      const long upper_off = DualWheelMapper::s_upper_sensor_offset_qc;
      const long lower_off = DualWheelMapper::s_lower_sensor_offset_qc;
      m_upper_qc.store(upper_off);
      m_lower_qc.store(lower_off);
      m_upper_sensor_state = SensorState::IDLE; // re-arm for first correction
      m_lower_sensor_state = SensorState::IDLE;
      // m_pos stays -1: mirrors real driver - wheels stop at sensor position,
      // not at any named slot. Operator must send setpos explicitly.
      m_homed.store(true);
      printf("[SIM][MockFilterWheel] Homing complete - send setpos to move to "
             "a slot\n");
    } else {
      printf("[SIM][MockFilterWheel] Homing halted\n");
    }
    m_moving.store(false);
  });

  return 0;
}

int MockFilterWheel::SetPosition(int virtual_slot) {
  if (virtual_slot < 0 || virtual_slot > 10)
    return 1;
  if (!m_homed.load())
    return 2;
  if (m_moving.load())
    return 3;

  join_worker();
  m_halt_requested.store(false);
  m_moving.store(true);
  m_pos.store(-1);

  m_worker = std::thread([this, virtual_slot]() {
    DualWheelMapper::PhysicalTarget target =
        DualWheelMapper::MapVirtualToPhysical(virtual_slot, m_upper_qc.load(),
                                              m_lower_qc.load());

    printf("[SIM][MockFilterWheel] Moving to virtual slot %d "
           "(upper_qc=%ld, lower_qc=%ld)\n",
           virtual_slot, target.upper_qc, target.lower_qc);

    const long upper_off = DualWheelMapper::s_upper_sensor_offset_qc;
    const long lower_off = DualWheelMapper::s_lower_sensor_offset_qc;
    // Both wheels move simultaneously - mirrors real hardware where each
    // drive runs independently on its own USB connection.
    bool ok_upper = false, ok_lower = false;
    std::thread t_lower([&]() {
      ok_lower = simulate_travel(m_lower_qc, m_lower_sensor_state,
                                 target.lower_qc, lower_off, 2U);
    });
    ok_upper = simulate_travel(m_upper_qc, m_upper_sensor_state,
                               target.upper_qc, upper_off, 1U);
    t_lower.join();

    if (ok_upper && ok_lower) {
      m_pos.store(virtual_slot);
      printf("[SIM][MockFilterWheel] Reached slot %d\n", virtual_slot);
    } else {
      printf("[SIM][MockFilterWheel] Move to slot %d halted\n", virtual_slot);
    }
    m_moving.store(false);
  });

  return 0;
}

int MockFilterWheel::GetPosition() const {
  if (m_moving.load())
    return -1;
  return m_pos.load();
}

bool MockFilterWheel::IsMoving() const {
  return m_moving.load();
}

bool MockFilterWheel::IsHomed() const {
  return m_homed.load();
}

int MockFilterWheel::Halt() {
  m_halt_requested.store(true);
  printf("[SIM][MockFilterWheel] Halt requested\n");
  return 0;
}

std::vector<std::string> MockFilterWheel::GetNames() const {
  return {"Clear",    "Filter 1", "Filter 2", "Filter 3",
          "Filter 4", "Filter 5", "Filter 6", "Filter 7",
          "Filter 8", "Filter 9", "Filter 10"};
}

long MockFilterWheel::GetUpperQC() const {
  return m_upper_qc.load();
}

long MockFilterWheel::GetLowerQC() const {
  return m_lower_qc.load();
}

int MockFilterWheel::GetFaultState() const {
  return 0;
} // no faults in simulation

} // namespace andor2k
