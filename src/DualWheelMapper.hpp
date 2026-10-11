#ifndef __DUALWHEELMAPPER_HPP__
#define __DUALWHEELMAPPER_HPP__

namespace andor2k {

/**
 * @brief Production logic for physical QC mapping and slot calculations.
 *
 * This class implements the mathematical "Digital Twin" of the physical
 * dual-wheel hardware. It ensures that both the simulation and the real Maxon
 * driver use the exact same logic for mapping virtual slots and calculating
 * forward-only quadrature counter (QC) targets.
 */
class DualWheelMapper {
public:
  // Tunable physical constants
  // QC_PER_SLOT is the primary measured value - update it after hardware
  // calibration; all other constants derive from it automatically.
  // Future migration to runtime config: replace this constexpr block with a
  // static WheelConfig load(const char *path) factory; every caller that
  // references DualWheelMapper::X will continue to compile unchanged.
  static constexpr long QC_PER_SLOT = 1433928L; // PRIMARY - calibrate this
  static constexpr int SLOTS_PER_WHEEL = 6;
  static constexpr long QC_PER_ROTATION =
      QC_PER_SLOT * SLOTS_PER_WHEEL; // derived

  // Homing sensor offset: QC stored in the accumulator when the home sensor
  // fires for the first time going forward from slot 0 (clear aperture).
  // Physical meaning: sensor flag is ~3 slot-spacings from slot 0.
  static constexpr long NORMAL_SENSOR_OFFSET = 3L * QC_PER_SLOT;
  // Reversed wheel (mirrored motor mount): forward encoder direction is
  // physically backward, so the sensor is approached from the opposite side.
  // The first rising edge occurs QC_PER_ROTATION - NORMAL_SENSOR_OFFSET into
  // the rotation from slot 0. For our geometry (sensor at 1/2 rotation) these
  // two values happen to be equal; keep them separate for post-calibration
  // adjustment (subtract sensor-flag geometry after hardware measurement).
  static constexpr long REVERSED_SENSOR_OFFSET =
      QC_PER_ROTATION - NORMAL_SENSOR_OFFSET;
  // Maximum allowable snap correction; larger drift invalidates home.
  static constexpr long MAX_DRIFT_QC = 100000L;

  // One or both motors may be mounted mirrored, reversing physical slot order
  // under positive velocity (0->5->4->3->2->1 instead of 0->1->2->3->4->5).
  // When true, MapVirtualToPhysical applies a forward-index reversal for that
  // wheel: physical slot k -> encoder index (SLOTS_PER_WHEEL-k)%6.
  //
  // Set the flag to false (or git revert the software-slot-reversal commit)
  // if the EPOS4 polarity bit 0x607E is configured on that drive instead.
  static constexpr bool UPPER_WHEEL_REVERSED = true;  // upper motor mirrored
  static constexpr bool LOWER_WHEEL_REVERSED = false; // lower motor normal
  struct PhysicalTarget {
    int upper_slot;
    int lower_slot;
    long upper_qc;
    long lower_qc;
  };

  /**
   * @brief Map a virtual slot index (0-10) to physical wheel targets.
   *
   * @param virtual_slot 0-10 (0: Clear/Clear, 1-5: Upper, 6-10: Lower)
   * @param current_upper_qc Current physical QC of the upper wheel.
   * @param current_lower_qc Current physical QC of the lower wheel.
   * @return PhysicalTarget containing target slots and forward-only QC values.
   */
  static PhysicalTarget MapVirtualToPhysical(int virtual_slot,
                                             long current_upper_qc,
                                             long current_lower_qc);

  /**
   * @brief Calculate the next QC target for a specific wheel using forward-only
   * logic.
   *
   * @param target_slot 0-5
   * @param current_qc Current physical QC position.
   * @return The next physical QC target (always >= current_qc).
   */
  static long CalculateForwardOnlyTarget(int target_slot, long current_qc);
};

} // namespace andor2k

#endif // __DUALWHEELMAPPER_HPP__
