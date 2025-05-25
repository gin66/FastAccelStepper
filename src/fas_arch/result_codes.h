#ifndef FAS_RESULT_CODES_H
#define FAS_RESULT_CODES_H

// ### Result codes for addQueueEntry() function of FastAccelStepper
enum class AqeResultCode : int8_t {
  OK = 0,
  QueueFull = 1,
  DirPinIsBusy = 2,
  WaitForEnablePinActive = 3,
  DeviceNotReady = 4,
  ErrorTicksTooLow = -1,
  ErrorEmptyQueueToStart = -2,
  ErrorNoDirPinToToggle = -3
};
static inline bool aqeRetry(AqeResultCode code) {
  return (static_cast<int8_t>(code)) > 0;
}
static inline const char* aqeToString(AqeResultCode code) {
  switch (code) {
    case AqeResultCode::OK:
      return "OK";
    case AqeResultCode::QueueFull:
      return "Queue Full";
    case AqeResultCode::DirPinIsBusy:
      return "Direction Pin is Busy";
    case AqeResultCode::WaitForEnablePinActive:
      return "Waiting for Enable Pin Active";
    case AqeResultCode::DeviceNotReady:
      return "Device Not Ready";
    case AqeResultCode::ErrorTicksTooLow:
      return "Error: Ticks Too Low";
    case AqeResultCode::ErrorEmptyQueueToStart:
      return "Error: Empty Queue to Start";
    case AqeResultCode::ErrorNoDirPinToToggle:
      return "Error: No Direction Pin to Toggle";
    default:
      return "Unknown Error";
  }
}
#define AQE_OK AqeResultCode::OK
#define AQE_QUEUE_FULL AqeResultCode::QueueFull
#define AQE_DIR_PIN_IS_BUSY AqeResultCode::DirPinIsBusy
#define AQE_WAIT_FOR_ENABLE_PIN_ACTIVE AqeResultCode::WaitForEnablePinActive
#define AQE_DEVICE_NOT_READY AqeResultCode::DeviceNotReady
#define AQE_ERROR_TICKS_TOO_LOW AqeResultCode::ErrorTicksTooLow
#define AQE_ERROR_EMPTY_QUEUE_TO_START AqeResultCode::ErrorEmptyQueueToStart
#define AQE_ERROR_NO_DIR_PIN_TO_TOGGLE AqeResultCode::ErrorNoDirPinToToggle

// Define the MoveResultCode enum with equivalent values
enum class MoveResultCode : int8_t {
  OK = 0,
  ErrorNoDirectionPin = -1,          // Equivalent to MOVE_ERR_NO_DIRECTION_PIN
  ErrorSpeedIsUndefined = -2,        // Equivalent to MOVE_ERR_SPEED_IS_UNDEFINED
  ErrorAccelerationIsUndefined = -3  // Equivalent to MOVE_ERR_ACCELERATION_IS_UNDEFINED
};

// Function to convert MoveResultCode to string for debugging/errors
static inline const char* moveToString(MoveResultCode code) {
  switch (code) {
    case MoveResultCode::OK:
      return "OK";
    case MoveResultCode::ErrorNoDirectionPin:
      return "Error: No Direction Pin";
    case MoveResultCode::ErrorSpeedIsUndefined:
      return "Error: Speed is Undefined";
    case MoveResultCode::ErrorAccelerationIsUndefined:
      return "Error: Acceleration is Undefined";
    default:
      return "Unknown Error";
  }
}

// Macros for easier access to enum values
#define MOVE_OK MoveResultCode::OK
#define MOVE_ERR_NO_DIRECTION_PIN MoveResultCode::ErrorNoDirectionPin
#define MOVE_ERR_SPEED_IS_UNDEFINED MoveResultCode::ErrorSpeedIsUndefined
#define MOVE_ERR_ACCELERATION_IS_UNDEFINED MoveResultCode::ErrorAccelerationIsUndefined

#endif /* FAS_RESULT_CODES_H */
