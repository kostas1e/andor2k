#ifndef __WHEELCONFIG_HPP__
#define __WHEELCONFIG_HPP__

#include <climits>

namespace andor2k {

/// Runtime-configurable wheel calibration and motion-profile constants.
///
/// Defaults match the compile-time constexpr values in DualWheelMapper.hpp
/// and MaxonFilterWheel.cpp. A plain-text config file can override any
/// subset of them; missing keys keep the default.
///
/// Config file format  (e.g. wheel.conf):
///   # comment line
///   key = value          # inline comment also supported
///   (blank lines ignored)
///
/// Typical usage:
///   WheelConfig cfg;
///   cfg.load("wheel.conf");          // false = file absent (use all defaults)
///   DualWheelMapper::configure(cfg); // update physics constants
///   wheel.configure(cfg);            // update motion profile
///
/// Safe to re-apply while wheels are idle (no move in progress).
/// Applying while moving: physics constants take effect at the NEXT move;
/// motion profile is re-programmed to the EPOS4 immediately (VCS call).
struct WheelConfig {
  // --- Physics constants (DualWheelMapper) ---------------------------------
  /// Full-rotation encoder count. PRIMARY measurement  - all slot targets
  /// are derived from this. Measure via RISE1-to-RISE1 sensor method.
  long qc_per_rotation = 8603568L;

  /// QC from clear (slot 0) to the RISE1 sensor event, lower wheel.
  /// Independently calibrated; NOT derived from qc_per_rotation.
  long lower_sensor_offset_qc = 4148391L; // 3xQC_PER_SLOT - 153393

  /// QC from clear (slot 0) to the RISE1 sensor event, upper wheel.
  /// Independently calibrated; see DualWheelMapper.hpp for asymmetry note.
  long upper_sensor_offset_qc =
      4168391L; // QC_PER_ROTATION - 3xQC_PER_SLOT - 133393

  /// Maximum allowable drift at each sensor crossing before home is
  /// invalidated. Must be > expected belt overshoot at operating speed.
  /// 100000 ~ 7% of one slot; appropriate for 4000 RPM.
  long max_drift_qc = 100000L;

  /// Half-width of the slot arrival window. Stops the forward-only wrap
  /// from triggering on a 1-QC overshoot. 10% of QC_PER_SLOT (~ 143392).
  long slot_tolerance_qc = 143392L;

  /// RISE2/RISE1 discriminator threshold. Sensor LOW-state QC below this
  /// = inter-pulse gap (RISE2). Above this = long clear-space (RISE1).
  /// Measured 2026-07-06: inter-pulse gap ~ 182k; clear space ~ 8.2M.
  long homing_gap_max_qc = 400000L;

  // --- Motion profile (programmed to EPOS4 via VCS_SetPositionProfile) -----
  unsigned int profile_velocity = 4000;      ///< rpm  - slot-to-slot moves
  unsigned int homing_velocity = 4000;       ///< rpm  - homing search run
  unsigned int profile_acceleration = 10000; ///< rpm/s
  unsigned int profile_deceleration = 15000; ///< rpm/s

  // --- Meta ----------------------------------------------------------------
  bool loaded = false; ///< true after a successful load() call

  /// Load from a plain-text key=value file.
  /// Returns false (not an error) if the file does not exist  - all defaults
  /// remain in effect. On parse error a warning is printed per bad line
  /// and the default is kept for that key.
  bool load(const char *path);

  /// Print all values to stdout with an optional label prefix.
  void print(const char *label = "WheelConfig") const;
};

} // namespace andor2k

#endif // __WHEELCONFIG_HPP__
