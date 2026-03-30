#pragma once

#include "intrinsic/utils/status.hpp"

namespace intrinsic::hal
{

// The hardware module interface defines the common contract between the
// hardware module runtime and a hardware module implementation.
//
// The hardware module implementation is responsible for reading and writing
// from/to hardware.
class HardwareModuleInterface
{
public:
  virtual ~HardwareModuleInterface() = default;

  // Initializes the hardware module.
  // This function is intended to be called only once and as part of the
  // initialization of the hardware module process.
  virtual Status Init() = 0;

  // Prepares the hardware module for activation.
  // This function is intended to be called as part of the connection phase
  // when the hardware module connects to the ICON IPC services.
  // All failures of `Prepare()` are considered fatal faults.
  virtual Status Prepare() { return OkStatus(); }

  // Activates the hardware module.
  // A call to `Activate()` signals the hardware module that ICON has
  // successfully connected to the hardware module and starts the realtime loop.
  // Prior to this, there's no call to `ReadStatus` happening.
  virtual RealtimeStatus Activate() = 0;

  // Deactivates the hardware module.
  // A call to `Deactivate()` signals the hardware module that ICON has been
  // disconnected and the hardware module is no longer controlled within a
  // realtime loop. No calls to `ReadStatus()` or `ApplyCommand()` are happening
  // after this.
  virtual RealtimeStatus Deactivate() = 0;

  // Enables motion commands for the hardware modules.
  // Returns kAborted to indicate a fatal fault.
  virtual Status EnableMotion() = 0;

  // Signals that the hardware module will now receive commands from ICON via
  // `ApplyCommand()`. Must return without blocking.
  virtual RealtimeStatus Enabled() { return RtOkStatus(); }

  // Signals that the hardware module will not receive commands anymore. Must
  // return without blocking.
  virtual RealtimeStatus Disabled() { return RtOkStatus(); }

  // Disables motion commands for the hardware modules.
  virtual Status DisableMotion() = 0;

  // Clear faults. This function can block and should only return with OkStatus
  // when the faults are cleared. Otherwise, it should return an error.
  virtual Status ClearFaults() = 0;

  // Shutdown the hardware module.
  virtual Status Shutdown() = 0;

  // Reads the current hardware status of all exported hardware interfaces.
  virtual RealtimeStatus ReadStatus() = 0;

  // Applies newly set commands of the hardware interfaces.
  virtual RealtimeStatus ApplyCommand() = 0;
};

}  // namespace intrinsic::hal
