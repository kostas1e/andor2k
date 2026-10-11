#ifndef __MOCKFILTERWHEEL_HPP__
#define __MOCKFILTERWHEEL_HPP__

#include "DualWheelMapper.hpp"
#include "IFilterWheel.hpp"

#include <atomic>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace andor2k {

/**
 * @brief Simulated filter wheel driver - no hardware required.
 *
 * Implements IFilterWheel using software-only simulation. A background
 * thread mimics the physical travel by incrementing QC values toward the
 * target, honouring Halt() stop requests. Uses DualWheelMapper for all
 * slot-to-QC mathematics so the sim and the real driver always agree.
 */
class MockFilterWheel : public IFilterWheel {
public:
  MockFilterWheel();
  ~MockFilterWheel() override;

  // Connection (no-ops in sim)
  int Connect() override;
  int Disconnect() override;

  // Slot control
  int SetPosition(int virtual_slot) override;
  int GetPosition() const override;

  // Motion state
  bool IsMoving() const override;
  bool IsHomed() const override;

  // Homing
  int GoHome() override;

  // Emergency stop
  int Halt() override;

  // Filter names (11 slots: 0=Clear, 1-5=Upper, 6-10=Lower)
  std::vector<std::string> GetNames() const override;

  // QC monitoring
  long GetUpperQC() const override;
  long GetLowerQC() const override;
  int  GetFaultState() const override;  // always 0 in simulation

  // Apply motion-profile parameters from WheelConfig. Geometry constants
  // (DualWheelMapper::s_*) are already updated by the caller before this.
  void configure(const WheelConfig &cfg) override;

private:
  // Simulated QC positions (accumulate; never reset)
  std::atomic<long> m_upper_qc{0};
  std::atomic<long> m_lower_qc{0};

  // Current virtual slot (-1 = unknown, -2 = moving)
  std::atomic<int> m_pos{-1};

  std::atomic<bool> m_moving{false};
  std::atomic<bool> m_homed{false};
  std::atomic<bool> m_halt_requested{false};

  // Background worker thread
  std::thread m_worker;

  void join_worker();

  // Index-pulse sensor simulation.
  // Each wheel flag has two tabs; the sensor fires on the leading edge of each
  // (RISE1 and RISE2). Only RISE1 is used for position correction; RISE2 is
  // logged informally and discarded by the state machine.
  enum class SensorState { IDLE, FIRST_PULSE, GAP, SECOND_PULSE };

  SensorState m_upper_sensor_state{SensorState::IDLE};
  SensorState m_lower_sensor_state{SensorState::IDLE};

  // Sensor flag geometry from measurements 2026-07-06 at 3000 RPM.
  // Upper (node 1): inter-pulse gap 173 244 QC; total flag span 448 422 QC.
  // Lower (node 2): inter-pulse gap 182 031 QC; total flag span 373 397 QC.
  // Tab widths estimated as (total_span - gap) / 2  (equal tabs assumed).
  static constexpr long SIM_TAB1_UPPER_QC = 137589L; // (448422 - 173244) / 2
  static constexpr long SIM_GAP_UPPER_QC  = 173244L; // measured
  static constexpr long SIM_TAB2_UPPER_QC = 137589L;
  static constexpr long SIM_TAB1_LOWER_QC =  95683L; // (373397 - 182031) / 2
  static constexpr long SIM_GAP_LOWER_QC  = 182031L; // measured
  static constexpr long SIM_TAB2_LOWER_QC =  95683L;

  // Velocity scale: measured 2026-07-06: 5/6 rotation (7 169 640 QC) ≈ 26 s
  // at 4000 RPM.  QC/s per RPM = 7 169 640 / 26 / 4000 ≈ 68.9.
  static constexpr double SIM_QCS_PER_RPM = 68.9;

  // Belt-jitter 1-sigma (QC). Produces realistic non-zero drift values in the
  // DI1^ log lines so --analyse drift statistics are meaningful.
  static constexpr double SIM_DRIFT_SIGMA_QC = 5000.0;

  // Simulation tick interval (microseconds).
  static constexpr int SIM_TICK_US = 5000;

  // Motion profile (from configure(); match WheelConfig defaults).
  unsigned int m_profile_velocity{4000};
  unsigned int m_homing_velocity{4000};
  unsigned int m_profile_accel{10000};
  unsigned int m_profile_decel{15000};

  // PRNG for belt-jitter drift simulation.
  std::mt19937 m_rng;
  std::normal_distribution<double> m_drift_dist{0.0, SIM_DRIFT_SIGMA_QC};

  // Simulate traveling from current QC to target using a trapezoidal velocity
  // profile (accel/decel from WheelConfig). Returns true if target reached,
  // false if halted mid-travel.
  bool simulate_travel(std::atomic<long> &qc, SensorState &sensor_state,
                       long target_qc, long sensor_offset,
                       unsigned short node_id);

  // Called each sim tick to advance the sensor state machine. Logs and applies
  // snap correction at RISE1; logs RISE2 informally without correction.
  void check_sensor(std::atomic<long> &qc, SensorState &state,
                    long prev_qc, long next_qc,
                    long sensor_offset, unsigned short node_id);
};

} // namespace andor2k

#endif // __MOCKFILTERWHEEL_HPP__
