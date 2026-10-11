#include "DualWheelMapper.hpp"

#include <cmath>

namespace andor2k {

DualWheelMapper::PhysicalTarget
DualWheelMapper::MapVirtualToPhysical(int virtual_slot, long current_upper_qc,
                                      long current_lower_qc) {
  PhysicalTarget target;

  // Mapping Logic:
  // Slot 0: Both Clear (Upper 0, Lower 0)
  // Slots 1-5: Upper Wheel moves (Lower stays Clear 0)
  // Slots 6-10: Lower Wheel moves (Upper stays Clear 0)

  if (virtual_slot <= 5) {
    target.upper_slot = virtual_slot;
    target.lower_slot = 0;
  } else {
    target.upper_slot = 0;
    target.lower_slot = (virtual_slot - 5); // 6->1, 7->2, etc.
  }

  // If a motor is mounted mirrored, physical slot k sits at encoder forward
  // index (SLOTS_PER_WHEEL - k) % SLOTS_PER_WHEEL. The reported slot in
  // target.{upper,lower}_slot (used for position feedback) is unchanged.
  int upper_fwd_index = UPPER_WHEEL_REVERSED
      ? (SLOTS_PER_WHEEL - target.upper_slot) % SLOTS_PER_WHEEL
      : target.upper_slot;
  int lower_fwd_index = LOWER_WHEEL_REVERSED
      ? (SLOTS_PER_WHEEL - target.lower_slot) % SLOTS_PER_WHEEL
      : target.lower_slot;
  target.upper_qc =
      CalculateForwardOnlyTarget(upper_fwd_index, current_upper_qc);
  target.lower_qc =
      CalculateForwardOnlyTarget(lower_fwd_index, current_lower_qc);

  return target;
}

long DualWheelMapper::CalculateForwardOnlyTarget(int target_slot,
                                                 long current_qc) {
  // 1. Determine current cycle and current relative position
  long current_cycle = current_qc / QC_PER_ROTATION;
  long current_pos_in_cycle = current_qc % QC_PER_ROTATION;

  // Handle potential negative remainder if current_qc somehow goes negative
  if (current_pos_in_cycle < 0) {
    current_pos_in_cycle += QC_PER_ROTATION;
    current_cycle -= 1;
  }

  // 2. Target relative position
  long target_pos_in_cycle = (long)target_slot * QC_PER_SLOT;

  // 3. Forward-only constraint with arrival tolerance.
  // diff > 0: we've overshot (current is ahead of target in this cycle).
  // diff < 0: we've undershot (current is behind target in this cycle).
  // |diff| <= SLOT_TOLERANCE_QC -> within the arrival window:
  //   return the in-cycle target so the delta sent to SetPosition is at most
  //   SLOT_TOLERANCE_QC. SetPosition clamps negative deltas (overshoot) to 0
  //   rather than reversing the motor, preventing the forward-only wrap that
  //   would otherwise cause a full extra rotation from a 1-QC overshoot.
  long diff = current_pos_in_cycle - target_pos_in_cycle;
  if (diff >= -SLOT_TOLERANCE_QC && diff <= SLOT_TOLERANCE_QC) {
    return current_cycle * QC_PER_ROTATION + target_pos_in_cycle;
  }
  if (target_pos_in_cycle < current_pos_in_cycle) {
    return (current_cycle + 1) * QC_PER_ROTATION + target_pos_in_cycle;
  } else {
    return current_cycle * QC_PER_ROTATION + target_pos_in_cycle;
  }
}

} // namespace andor2k
