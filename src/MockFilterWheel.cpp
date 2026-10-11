#include "MockFilterWheel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

using namespace std::chrono_literals;

namespace andor2k {

// Position along a trapezoidal velocity profile at time t [s].
// dist_qc = total distance; v_max/a/d in QC/s and QC/s².
// Returns clamped to [0, dist_qc]; safe to call with t beyond move end.
static double sim_qc_at_time(double t, double dist_qc,
                             double v_max, double a, double d) {
  const double d_a = v_max * v_max / (2.0 * a);
  const double d_d = v_max * v_max / (2.0 * d);
  double pos;
  if (d_a + d_d <= dist_qc) {
    // Full trapezoidal: accel → constant → decel
    const double t_a = v_max / a;
    const double t_c = (dist_qc - d_a - d_d) / v_max;
    if (t <= t_a)
      pos = 0.5 * a * t * t;
    else if (t <= t_a + t_c)
      pos = d_a + v_max * (t - t_a);
    else {
      const double tau = t - t_a - t_c;
      pos = d_a + v_max * t_c + v_max * tau - 0.5 * d * tau * tau;
    }
  } else {
    // Triangle: never reaches v_max
    const double v_pk = std::sqrt(2.0 * dist_qc * a * d / (a + d));
    const double t_a  = v_pk / a;
    if (t <= t_a)
      pos = 0.5 * a * t * t;
    else {
      const double tau = t - t_a;
      pos = 0.5 * a * t_a * t_a + v_pk * tau - 0.5 * d * tau * tau;
    }
  }
  return std::min(dist_qc, std::max(0.0, pos));
}

MockFilterWheel::MockFilterWheel() : m_rng(std::random_device{}()) {}

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

void MockFilterWheel::configure(const WheelConfig &cfg) {
  m_profile_velocity = cfg.profile_velocity;
  m_homing_velocity  = cfg.homing_velocity;
  m_profile_accel    = cfg.profile_acceleration;
  m_profile_decel    = cfg.profile_deceleration;
  printf("[SIM][MockFilterWheel] configure: vel=%u hom_vel=%u accel=%u decel=%u RPM(s)\n",
         m_profile_velocity, m_homing_velocity, m_profile_accel, m_profile_decel);
}

int MockFilterWheel::Disconnect() {
  join_worker();
  printf("[SIM][MockFilterWheel] Disconnect() - no-op\n");
  return 0;
}

bool MockFilterWheel::simulate_travel(std::atomic<long> &qc,
                                      SensorState &sensor_state, long target_qc,
                                      long sensor_offset,
                                      unsigned short node_id) {
  const long dist_qc = target_qc - qc.load();
  if (dist_qc <= 0) return true;

  // Convert motion profile (RPM, RPM/s) to QC-based units.
  const double v_max = m_profile_velocity * SIM_QCS_PER_RPM; // QC/s
  const double a     = m_profile_accel    * SIM_QCS_PER_RPM; // QC/s²
  const double d     = m_profile_decel    * SIM_QCS_PER_RPM; // QC/s²

  // Trapezoidal total move time.
  const double d_a = v_max * v_max / (2.0 * a);
  const double d_d = v_max * v_max / (2.0 * d);
  double total_s;
  if (d_a + d_d <= dist_qc)
    total_s = v_max / a + (dist_qc - d_a - d_d) / v_max + v_max / d;
  else {
    const double v_pk = std::sqrt(2.0 * dist_qc * a * d / (a + d));
    total_s = v_pk / a + v_pk / d;
  }
  printf("[SIM][MockFilterWheel] node %u: dist=%ld QC total_s=%.2fs\n",
         node_id, dist_qc, total_s);

  const double dt       = SIM_TICK_US / 1.0e6;
  const long start_qc   = qc.load();
  double t = 0.0;

  while (t < total_s) {
    if (m_halt_requested.load()) return false;
    t += dt;
    if (t > total_s) t = total_s;

    long prev_val = qc.load();
    long next_val = start_qc + (long)sim_qc_at_time(t, (double)dist_qc, v_max, a, d);
    if (next_val > target_qc) next_val = target_qc;
    qc.store(next_val);
    check_sensor(qc, sensor_state, prev_val, next_val, sensor_offset, node_id);
    std::this_thread::sleep_for(std::chrono::microseconds(SIM_TICK_US));
  }
  qc.store(target_qc);
  return true;
}

void MockFilterWheel::check_sensor(std::atomic<long> &qc, SensorState &state,
                                   long prev_qc, long next_qc,
                                   long sensor_offset, unsigned short node_id) {
  // Per-node sensor flag geometry (measurements 2026-07-06 at 3000 RPM).
  const long tab1 = (node_id == 1) ? SIM_TAB1_UPPER_QC : SIM_TAB1_LOWER_QC;
  const long gap  = (node_id == 1) ? SIM_GAP_UPPER_QC  : SIM_GAP_LOWER_QC;
  const long tab2 = (node_id == 1) ? SIM_TAB2_UPPER_QC : SIM_TAB2_LOWER_QC;
  const long rise2_off = tab1 + gap;   // RISE2 position from RISE1 within cycle
  const long end_off   = rise2_off + tab2; // trailing edge of tab2

  switch (state) {
  case SensorState::IDLE: {
    const long prev_cycle =
        (prev_qc - sensor_offset) / DualWheelMapper::s_qc_per_rotation;
    const long next_cycle =
        (next_qc - sensor_offset) / DualWheelMapper::s_qc_per_rotation;
    if (next_cycle > prev_cycle) {
      // RISE1: snap position and log drift with Gaussian belt jitter.
      const long expected_qc =
          next_cycle * DualWheelMapper::s_qc_per_rotation + sensor_offset;
      const long current = qc.load();
      const long jitter  = (long)m_drift_dist(m_rng);
      const long drift   = (current - expected_qc) + jitter;
      printf("[SIM][MockFilterWheel] DI1^ node %u: QC %ld -> %ld (drift %+ld QC)\n",
             node_id, current, expected_qc, drift);
      if (std::abs(drift) < DualWheelMapper::s_max_drift_qc) {
        qc.store(expected_qc);
      } else {
        printf("[SIM][MockFilterWheel] WARNING node %u: drift %ld QC "
               "exceeds threshold - home invalidated\n",
               node_id, std::abs(drift));
        m_homed.store(false);
      }
      state = SensorState::FIRST_PULSE;
    }
    break;
  }
  case SensorState::FIRST_PULSE: {
    const long pos =
        (next_qc - sensor_offset) % DualWheelMapper::s_qc_per_rotation;
    if (pos >= tab1)
      state = SensorState::GAP;
    break;
  }
  case SensorState::GAP: {
    const long pos =
        (next_qc - sensor_offset) % DualWheelMapper::s_qc_per_rotation;
    if (pos >= rise2_off) {
      // RISE2: log for realism; not used for position correction.
      printf("[SIM][MockFilterWheel] DI1^ node %u RISE2 at +%ld QC "
             "(inter-pulse gap, not corrected)\n",
             node_id, rise2_off);
      state = SensorState::SECOND_PULSE;
    }
    break;
  }
  case SensorState::SECOND_PULSE: {
    const long pos =
        (next_qc - sensor_offset) % DualWheelMapper::s_qc_per_rotation;
    if (pos >= end_off)
      state = SensorState::IDLE;
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
    // Approximate homing time: assume half-rotation worst-case travel
    // at homing velocity (same SIM_QCS_PER_RPM scale as SetPosition).
    const long home_dist_qc = DualWheelMapper::s_qc_per_rotation / 2;
    const double v_home     = m_homing_velocity * SIM_QCS_PER_RPM;
    const int home_ms = std::max(200, (int)(1000.0 * home_dist_qc / v_home));
    std::this_thread::sleep_for(std::chrono::milliseconds(home_ms));

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
