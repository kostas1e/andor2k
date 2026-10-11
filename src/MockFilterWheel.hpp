#ifndef __MOCKFILTERWHEEL_HPP__
#define __MOCKFILTERWHEEL_HPP__

#include "DualWheelMapper.hpp"
#include "IFilterWheel.hpp"

#include <atomic>
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

  // Index-pulse sensor simulation
  // Each wheel has one sensor that fires TWICE per revolution (hardware quirk:
  // the magnet/metal flag is wide enough to trigger the sensor on approach and
  // departure). Only the FIRST trigger of each pair is used for position
  // correction; the second is discarded by the state machine.
  enum class SensorState { IDLE, FIRST_PULSE, GAP, SECOND_PULSE };

  SensorState m_upper_sensor_state{SensorState::IDLE};
  SensorState m_lower_sensor_state{SensorState::IDLE};

  // Simulated sensor zone widths (QC units within one rotation).
  // First pulse starts at each rotation boundary (QC % QC_PER_ROTATION == 0).
  static constexpr long SIM_SENSOR_PULSE1_QC = 50000L;
  static constexpr long SIM_SENSOR_GAP_QC = 25000L;
  static constexpr long SIM_SENSOR_PULSE2_QC = 50000L;
  // If the snap correction would exceed this threshold, homing is invalidated.
  static constexpr long SIM_MAX_DRIFT_QC = 100000L;

  // Simulate traveling from current QC to target, stepping in SIM_QC_STEP
  // increments. Applies sensor-based index correction when a rotation boundary
  // is crossed. Returns true if target reached, false if halted.
  // node_id: 1 = upper wheel, 2 = lower wheel (used in log output).
  bool simulate_travel(std::atomic<long> &qc, SensorState &sensor_state,
                       long target_qc, long sensor_offset,
                       unsigned short node_id, int step_us = 5000);

  // Called each step to advance the sensor state machine and apply position
  // correction when the first trigger of a double-pulse pair is detected.
  // node_id: 1 = upper wheel, 2 = lower wheel (used in log output).
  void check_sensor(std::atomic<long> &qc, SensorState &state,
                    long prev_qc, long next_qc,
                    long sensor_offset, unsigned short node_id);
};

} // namespace andor2k

#endif // __MOCKFILTERWHEEL_HPP__
