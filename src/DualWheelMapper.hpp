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
  // QC_PER_ROTATION is the primary measured value - update it after hardware
  // calibration using the RISE1-to-RISE1 sensor method (see commissioning
  // notes in andor2k_cookbook.tex). QC_PER_SLOT is derived; integer division
  // is exact when the wheel has uniform slot spacing (designed-in property).
  // Future migration to runtime config: replace this constexpr block with a
  // static WheelConfig load(const char *path) factory; every caller that
  // references DualWheelMapper::X will continue to compile unchanged.
  static constexpr int SLOTS_PER_WHEEL = 6;
  static constexpr long QC_PER_ROTATION =
      8603568L; // PRIMARY - measure via RISE1-to-RISE1
  static constexpr long QC_PER_SLOT =
      QC_PER_ROTATION / SLOTS_PER_WHEEL; // derived

  // DIRECTION CONVENTION (defined from positive motor velocity):
  //
  //   Lower wheel (Node 2) positive velocity -> QC+ -> slots:
  //   Clear->6->7->8->9->10
  //                         This is the REFERENCE direction.
  //                         LOWER_WHEEL_REVERSED = false.
  //
  //   Upper wheel (Node 1) positive velocity -> QC+ -> slots:
  //   Clear->5->4->3->2->1
  //                         Opposite physical direction to lower.
  //                         UPPER_WHEEL_REVERSED = true.
  //
  // SENSOR OFFSET CONVENTION:
  //
  //   LOWER_SENSOR_OFFSET_QC = QC accumulated going FORWARD (reference dir)
  //     from slot 0 (clear) until RISE1 fires.
  //     RISE1 = leading edge of flag tab 1 enters sensor.
  //
  //   UPPER_SENSOR_OFFSET_QC = QC accumulated going FORWARD in upper wheel's
  //     frame (opposite physical direction) from slot 0 until RISE1 fires.
  //     RISE1 = trailing edge of flag tab 2 enters sensor from opposite side.
  //
  //   These are NOT related by UPPER = QC_PER_ROTATION - LOWER.
  //   The initial values happen to be equal (both = QC_PER_ROTATION/2 for a
  //   6-slot wheel with sensor near 180 deg) but they diverge after calibration
  //   because the two RISE1 events trigger at different physical positions
  //   (offset ~tab_width + inter-pulse gap) and the two assemblies have
  //   independent sensor mounting tolerances.
  //
  //   CALIBRATION RULE (same for both wheels):
  //     After home + setpos N, inspect physical alignment.
  //     Filter/aperture PAST centre -> wheel went too far -> INCREASE
  //     offset.
  //     Filter/aperture BEFORE centre -> wheel stopped short -> DECREASE
  //     offset.
  //     See commissioning.md Sec.8 for per-wheel procedure.
  static constexpr long LOWER_SENSOR_OFFSET_QC =
      3L * QC_PER_SLOT - 153393L; // cal 2026-07-06 v2: stopped ~1/10 slot
                                  // (143 393 QC) early -> -143 393
  static constexpr long UPPER_SENSOR_OFFSET_QC =
      QC_PER_ROTATION - 3L * QC_PER_SLOT -
      133393L; // cal 2026-07-06 v2: same -143 393 applied

  // Legacy aliases - kept so existing references compile unchanged.
  // Prefer the per-wheel names above for all new or modified code.
  static constexpr long NORMAL_SENSOR_OFFSET = LOWER_SENSOR_OFFSET_QC;
  static constexpr long REVERSED_SENSOR_OFFSET = UPPER_SENSOR_OFFSET_QC;
  // Maximum allowable snap correction; larger drift invalidates home.
  static constexpr long MAX_DRIFT_QC = 100000L;

  // Maximum LOW-state QC that can be the inter-pulse gap (< this = gap/RISE2,
  // > this = long clear space = RISE1).
  // Measured 2026-07-06 at 3000 RPM:
  //   Upper (node 1) inter-pulse gap: 173 244 QC (Tab2->gap->Tab1 in upper
  //   dir) Lower (node 2) inter-pulse gap: 182 031 QC (Tab1->gap->Tab2 in
  //   lower dir) Upper clear space: 8 155 146 QC (Tab1+Tab2 combined = 448 422
  //   QC) Lower clear space: 8 230 171 QC (Tab1+Tab2 combined = 373 397 QC)
  // Threshold set to 400 000 QC: > 2* largest gap, well below clear space.
  static constexpr long HOMING_GAP_MAX_QC = 400000L;

  // Arrival tolerance for slot positioning. If the wheel stops within this
  // many QC of a slot center (in either direction) the position is accepted
  // without issuing a corrective or a full extra rotation. This absorbs belt
  // overshoot/undershoot from the rubber-band drive without triggering the
  // forward-only wrap (overshoot by 1 QC -> full extra rotation).
  // 10% of QC_PER_SLOT ~6 deg of wheel rotation. Tune after measuring real
  // belt overshoot at operating speed. Must be << QC_PER_SLOT / 2 so the
  // tolerance zone of adjacent slots never overlap.
  static constexpr long SLOT_TOLERANCE_QC = QC_PER_SLOT / 10; // ~143 393 QC

  // Reversal flags - see direction convention above.
  // Set UPPER_WHEEL_REVERSED = false (and re-calibrate UPPER_SENSOR_OFFSET_QC)
  // if the EPOS4 encoder polarity bit (0x607E bit 0) is flipped on the upper
  // drive in EPOS Studio instead of using software reversal.
  static constexpr bool UPPER_WHEEL_REVERSED =
      true; // positive V -> slots Clear->5->4->3->2->1
  static constexpr bool LOWER_WHEEL_REVERSED =
      false; // positive V -> slots Clear->6->7->8->9->10
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
