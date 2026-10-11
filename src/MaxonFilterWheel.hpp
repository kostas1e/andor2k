#ifndef __MAXONFILTERWHEEL_HPP__
#define __MAXONFILTERWHEEL_HPP__

#ifndef MOCK_WHEEL
#include "DualWheelMapper.hpp"
#include "IFilterWheel.hpp"

#include <atomic>
#include <string>
#include <thread>
#include <vector>

namespace andor2k {

/**
 * @brief Production filter wheel driver using Maxon EPOS4 via EposCmd USB API.
 *
 * Both wheels are driven by Maxon EPOS4 Compact units connected via USB.
 * Node IDs: upper wheel = 1, lower wheel = 2.
 *
 * Position tracking: VCS_GetPositionIs() returns a 32-bit int (+/-2^31 QC).
 * Because the wheels accumulate position in forward-only mode the counters will
 * grow beyond int range over time. We therefore maintain separate 64-bit
 * accumulators that are updated by detecting int wrap-around.
 */
class MaxonFilterWheel : public IFilterWheel {
public:
  MaxonFilterWheel();
  ~MaxonFilterWheel() override;

  int Connect() override;
  int Disconnect() override;

  int SetPosition(int virtual_slot) override;
  int GetPosition() const override;

  bool IsMoving() const override;
  bool IsHomed() const override;

  int GoHome() override;
  int Halt() override;

  std::vector<std::string> GetNames() const override;

  long GetUpperQC() const override;
  long GetLowerQC() const override;
  int  GetFaultState() const override;  // bit 0=upper, bit 1=lower

private:
  // EPOS4 device handles - one per physical USB connection (nullptr when
  // disconnected). Drives are identified by node ID, NOT by USB enumeration
  // order. IMPORTANT: configure these node IDs in EPOS Studio before first use:
  //   upper wheel drive -> Node ID = NODE_UPPER (1)
  //   lower wheel drive -> Node ID = NODE_LOWER (2)
  void *m_handle_upper = nullptr;
  void *m_handle_lower = nullptr;

  static constexpr unsigned short NODE_UPPER = 1; // EPOS Studio: upper wheel
  static constexpr unsigned short NODE_LOWER = 2; // EPOS Studio: lower wheel

  // 64-bit accumulated QC positions (updated from 32-bit VCS_GetPositionIs)
  std::atomic<long> m_upper_qc{0};
  std::atomic<long> m_lower_qc{0};

  // Last raw 32-bit readings (for wraparound detection)
  int m_upper_raw_prev = 0;
  int m_lower_raw_prev = 0;

  // DI1-based index-pulse correction
  // The sensor wired to DI1 (EPOS4 digital input 1, object 0x60FD bit 16) fires
  // TWICE per revolution due to the double-width magnet flag. Only the first
  // rising edge of each pair is used for position correction; the second is
  // discarded by the state machine.
  // Counter is NOT reset to 0 - it is snapped to the nearest rotation boundary
  // (N x QC_PER_ROTATION) to remove accumulated drift without disrupting the
  // forward-only movement target that was already computed.
  enum class SensorState { IDLE, FIRST_PULSE, GAP, SECOND_PULSE };

  SensorState m_upper_sensor_state{SensorState::IDLE};
  SensorState m_lower_sensor_state{SensorState::IDLE};

  // Previous DI1 levels - used to detect rising/falling edges.
  bool m_upper_di1_prev{false};
  bool m_lower_di1_prev{false};

  // Software homing constants
  // TODO(cal): run the commissioning procedure below on first hardware use.
  // QC value written to the EPOS4 position register (via 0x607C home offset
  // + VCS_FindHome HM_ACTUAL_POSITION) when each wheel's sensor fires.
  // Clear slot (virtual 0) lives at multiples of QC_PER_ROTATION.
  // After homing: GetUpperQC() / GetLowerQC() return this offset.
  // SetPosition(0) will then move the wheel the remaining
  //   (QC_PER_ROTATION - offset) QC to reach the first clear-slot.
  //
  //   1. Home the wheels.
  //   2. Press Go -> slot 0 (clear).
  //   3. If the wheel does not land on the clear aperture, measure the
  //      physical misalignment in QC (from the daemon log) and adjust.
  //   NOTE: The sensor fires TWICE per revolution (hardware flaw).
  //         Software homes on the FIRST rising edge seen going forward.
  //         If the wrong trigger is caught, add QC_PER_ROTATION / 2
  //         (~4 303 284 QC) to each offset.
  //   These constants will be exported to a config file in a later revision.
  // Per-wheel sensor offsets - reference the independently-calibrated constants
  // in DualWheelMapper. Do NOT derive one from the other (see
  // DualWheelMapper.hpp).
  static constexpr long UPPER_HOMING_SENSOR_OFFSET_QC =
      DualWheelMapper::UPPER_SENSOR_OFFSET_QC;
  static constexpr long LOWER_HOMING_SENSOR_OFFSET_QC =
      DualWheelMapper::LOWER_SENSOR_OFFSET_QC;

  std::atomic<int> m_pos{-1};
  std::atomic<bool> m_moving{false};
  std::atomic<bool> m_homed{false};
  std::atomic<bool> m_halt_requested{false};

  std::thread m_worker;

  void join_worker();

  // Clear faults, enable, and activate profile position mode on one node.
  int init_node(void *handle, unsigned short node_id);

  // Update the 64-bit accumulator for one wheel from the latest 32-bit reading.
  void update_qc(std::atomic<long> &acc, int &prev_raw, void *handle,
                 unsigned short node_id);

  // Home one wheel by moving forward at PROFILE_VELOCITY until DI1 fires,
  // then setting the EPOS4 position register to sensor_offset via 0x607C /
  // VCS_FindHome(HM_ACTUAL_POSITION). Returns true on success.
  // Respects m_halt_requested; on halt returns false without leaving motor
  // running.
  bool home_one_wheel(void *handle, unsigned short node,
                      std::atomic<long> &qc_ref, int &raw_prev_ref,
                      long sensor_offset) noexcept;

  // Move forward (velocity mode already active) until the home sensor fires
  // (DI1 rising edge, bit 2 of 0x60FD). If DI1 is HIGH on entry, waits for
  // it to fall first. Returns true on success; fills low_qc_before_rise with
  // the QC elapsed during the LOW state before this rising edge.
  bool seek_next_rise(void *handle, unsigned short node,
                      std::atomic<long> &qc_ref, int &raw_prev,
                      long &low_qc_before_rise,
                      std::chrono::steady_clock::time_point deadline) noexcept;

  // Read DI1 from the EPOS4 digital inputs object (0x60FD bit 16), detect
  // rising/falling edges, and apply position correction on the first trigger
  // of each double-pulse pair.
  void poll_sensor(void *handle, unsigned short node_id, std::atomic<long> &qc,
                   SensorState &state, bool &di1_prev, long sensor_offset);
};

} // namespace andor2k

#endif // MOCK_WHEEL

#endif // __MAXONFILTERWHEEL_HPP__
