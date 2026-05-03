
#include "MotionManager.hpp"
namespace motion_manager {

MotionManager::MotionManager(ReadObjectFunc read_object,
                  InputReadFunc read_inputs,
                  DriveDisableFunc disable_drive,
                  DriveEnableFunc enable_drive)
  : read_object_(read_object),
    read_inputs_(read_inputs),
    disable_drive_(disable_drive),
    enable_drive_(enable_drive) {}


auto MotionManager::Initialize(motion_profile::TargetConstraints linear_constraints) -> void {}
auto MotionManager::RequestMode(uint8_t new_mode) -> MotionModes {
    switch (static_cast<MotionModes>(new_mode)) {
      case MotionModes::kNone:
      case MotionModes::kProfilePositionMode:
        mode_impl_=motion_profile::TrapezoidalRamp{motion_profile::TargetConstraints{}};
      case MotionModes::kProfileVelocityMode:
        mode_impl_=motion_profile::ConstantVelocityProfile{};
      case MotionModes::kTorqueProfileMode:
      case MotionModes::kHomingMode:
      mode_impl_=motion_profile::homing::HomingController{read_object_,read_inputs_,disable_drive_,enable_drive_};
      case MotionModes::kCylicPositionMode:
      case MotionModes::kCylicVelocityMode:
      case MotionModes::kCylicProfileMode:
        break;
    }
  mode_ = static_cast<MotionModes>(new_mode);
  return mode_;
}
auto MotionManager::GetFrequency(int t) -> double {
  return std::visit([t](auto& d) -> double { return d.GetFrequency(t); }, mode_impl_);
}

}  // namespace motion_manager