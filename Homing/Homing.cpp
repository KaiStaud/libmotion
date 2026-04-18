#include "Homing.hpp"

namespace motion_profile::homing {
HomingController::HomingController(ReadObjectFunc read_object,
                   InputReadFunc read_inputs,
                   DriveDisableFunc disable_drive,
                   DriveEnableFunc enable_drive)
    : read_object_(read_object),
      read_inputs_(read_inputs),
      disable_drive_(disable_drive),
      enable_drive_(enable_drive) {}

auto HomingController::Update() -> Progress {
  switch (state_) {
    case Progress::kDisabled:
      if (read_object_ &&
          read_object_(kIdxModeOfOperation, 0) == kModeHoming) {
        state_ = Progress::kOpModeConfigured;
      }
      break;

    case Progress::kOpModeConfigured:
      if (read_object_ &&
          read_object_(kIdxHomingMethod, 0) == kHomingMethodDefault) {
        state_ = Progress::kProfileConfigured;
      }
      break;

    case Progress::kProfileConfigured:
      if (read_object_ &&
          read_object_(kIdxModeOfOperation, 0) == kModeHomingStart) {
        state_ = Progress::kStarted;
      }
      break;

    case Progress::kStarted:
      enable_drive_();
      state_ = Progress::kActive;
      break;

    case Progress::kActive:
      if (read_inputs_()) {
        state_ = Progress::kDone;
      }
      break;

    case Progress::kDone:
    disable_drive_();
      // Optional cleanup
      break;
  }

  return state_;
}

Progress HomingController::state() const {
  return state_;
}

}  // namespace motion_profile::homing
