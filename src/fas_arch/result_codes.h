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
static inline bool aqeIsOk(AqeResultCode status) {
  return status == AqeResultCode::OK;
}

static inline const char* toString(AqeResultCode code) {
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
  ErrorNoDirectionPin = -1,    // Equivalent to MOVE_ERR_NO_DIRECTION_PIN
  ErrorSpeedIsUndefined = -2,  // Equivalent to MOVE_ERR_SPEED_IS_UNDEFINED
  ErrorAccelerationIsUndefined =
      -3  // Equivalent to MOVE_ERR_ACCELERATION_IS_UNDEFINED
};

static inline bool moveIsOk(MoveResultCode status) {
  return status == MoveResultCode::OK;
}

// Function to convert MoveResultCode to string for debugging/errors
static inline const char* toString(MoveResultCode code) {
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
#define MOVE_ERR_ACCELERATION_IS_UNDEFINED \
  MoveResultCode::ErrorAccelerationIsUndefined

// Define the MoveResultCode enum with equivalent values
enum class MoveTimedResultCode : int8_t {
  OK = 0,
  QueueFull = 1,
  DirPinIsBusy = 2,
  WaitForEnablePinActive = 3,
  DeviceNotReady = 4,
  ErrorTicksTooLow = -1,
  ErrorEmptyQueueToStart = -2,
  ErrorNoDirPinToToggle = -3,
  MoveBusy = 5,
  MoveEmpty = 6,
  ErrorMoveTooLarge = -4,
};

static inline bool moveTimedIsOk(MoveTimedResultCode status) {
  return status == MoveTimedResultCode::OK;
}

static inline MoveTimedResultCode tmrFrom(AqeResultCode res) {
  return static_cast<MoveTimedResultCode>(res);
}

// Function to convert MoveResultCode to string for debugging/errors
static inline const char* toString(MoveTimedResultCode code) {
  switch (code) {
    case MoveTimedResultCode::OK:
      return "OK";
    case MoveTimedResultCode::QueueFull:
      return "Queue Full";
    case MoveTimedResultCode::DirPinIsBusy:
      return "Direction Pin is Busy";
    case MoveTimedResultCode::WaitForEnablePinActive:
      return "Waiting for Enable Pin Active";
    case MoveTimedResultCode::DeviceNotReady:
      return "Device Not Ready";
    case MoveTimedResultCode::ErrorTicksTooLow:
      return "Error: Ticks Too Low";
    case MoveTimedResultCode::ErrorEmptyQueueToStart:
      return "Error: Empty Queue to Start";
    case MoveTimedResultCode::ErrorNoDirPinToToggle:
      return "Error: No Direction Pin to Toggle";
    case MoveTimedResultCode::MoveBusy:
      return "Move still ongoing";
    case MoveTimedResultCode::MoveEmpty:
      return "Queue has been empty";
    case MoveTimedResultCode::ErrorMoveTooLarge:
      return "Error: Move too large";
    default:
      return "Unknown Error";
  }
}
#define MOVE_TIMED_OK MoveTimedResultCode::OK
#define MOVE_TIMED_BUSY MoveTimedResultCode::MoveBusy
#define MOVE_TIMED_EMPTY MoveTimedResultCode::MoveEmpty
#define MOVE_TIMED_TOO_LARGE_ERROR MoveTimedResultCode::ErrorMoveTooLarge

enum class DelayResultCode : int8_t { OK = 0, TOO_LOW = -1, TOO_HIGH = -2 };

static inline bool delayIsValid(DelayResultCode status) {
  return status == DelayResultCode::OK;
}

static inline const char* toString(DelayResultCode status) {
  switch (status) {
    case DelayResultCode::OK:
      return "OK";
    case DelayResultCode::TOO_LOW:
      return "Delay Too Low";
    case DelayResultCode::TOO_HIGH:
      return "Delay Too High";
    default:
      return "Unknown Delay Status";
  }
}

// Macros for simpler usage if needed
#define DELAY_OK DelayResultCode::OK
#define DELAY_TOO_LOW DelayResultCode::TOO_LOW
#define DELAY_TOO_HIGH DelayResultCode::TOO_HIGH

#endif /* FAS_RESULT_CODES_H */
