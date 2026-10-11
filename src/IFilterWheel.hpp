#ifndef __IFILTERWHEEL_HPP__
#define __IFILTERWHEEL_HPP__

#include "WheelConfig.hpp"

#include <string>
#include <vector>

namespace andor2k {

/**
 * @brief ASCOM-aligned interface for Filter Wheel control.
 *
 * This interface defines the core functionality required to manage a filter
 * wheel system, aligned with ASCOM standards while providing hooks for physical
 * quadrature counter (QC) monitoring.
 */
class IFilterWheel {
public:
  virtual ~IFilterWheel() = default;

  /**
   * @brief Open the connection to the hardware (or initialize the sim).
   * @return 0 on success, non-zero error code otherwise.
   */
  virtual int Connect() = 0;

  /**
   * @brief Close the connection and release resources.
   * @return 0 on success, non-zero error code otherwise.
   */
  virtual int Disconnect() = 0;

  /**
   * @brief Move the wheel to a specific slot index.
   * @param position 0-based index of the filter slot.
   * @return 0 on success, non-zero error code otherwise.
   */
  virtual int SetPosition(int position) = 0;

  /**
   * @brief Get the current filter slot index.
   * @return 0-based index of the current slot, or -1 if position is unknown.
   */
  virtual int GetPosition() const = 0;

  /**
   * @brief Check if the wheel is currently performing a move operation.
   * @return true if moving, false if idle.
   */
  virtual bool IsMoving() const = 0;

  /**
   * @brief Check if the wheel has been calibrated (homed) since power-on.
   * @return true if homed, false if homing is required.
   */
  virtual bool IsHomed() const = 0;

  /**
   * @brief Start the homing (calibration) procedure.
   * This operation is usually asynchronous.
   * @return 0 if homing started, non-zero error code otherwise.
   */
  virtual int GoHome() = 0;

  /**
   * @brief Immediately stop all movement.
   * @return 0 on success.
   */
  virtual int Halt() = 0;

  /**
   * @brief Get the human-readable names of the filters.
   * @return Vector of filter names corresponding to slot indices.
   */
  virtual std::vector<std::string> GetNames() const = 0;

  /**
   * @brief Get the current physical Quadrature Counter (QC) value for the
   * upper wheel.
   * @return Current QC value.
   */
  virtual long GetUpperQC() const = 0;

  /**
   * @brief Get the current physical Quadrature Counter (QC) value for the
   * lower wheel.
   * @return Current QC value.
   */
  virtual long GetLowerQC() const = 0;

  /**
   * @brief Query drive fault state.
   * @return Bitmask: bit 0 = upper drive fault, bit 1 = lower drive fault.
   *         0 means no faults. Simulation always returns 0.
   */
  virtual int GetFaultState() const = 0;

  /**
   * @brief Apply calibration and motion-profile constants from a WheelConfig.
   *
   * Default no-op for implementations that do not support runtime
   * reconfiguration (e.g. MockFilterWheel). MaxonFilterWheel overrides this
   * to reprogramme the EPOS4 position profile.
   *
   * Note: physics constants (DualWheelMapper::s_*) are always updated by the
   * caller (andor2kd apply_wheel_config()) before this is called.
   */
  virtual void configure(const WheelConfig &cfg) {
    (void)cfg;
  }
};

} // namespace andor2k

#endif // __IFILTERWHEEL_HPP__
