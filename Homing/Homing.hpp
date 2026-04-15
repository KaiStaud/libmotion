#pragma once
#include <cstdint>

namespace motion_profile::homing {

enum class Progress : uint8_t {
  kDisabled,
  kOpModeConfigured,
  kProfileConfigured,
  kStarted,
  kActive,
  kDone
};

class HomingController {
 public:
  using InputReadFunc = bool (*)();
  using DriveEnableFunc = void (*)();
  using DriveDisableFunc = void (*)();
  using ReadObjectFunc = uint32_t (*)(uint16_t index, uint8_t subindex);

  HomingController(ReadObjectFunc read_object,
                   InputReadFunc read_gpio,
                   DriveDisableFunc disable_drive,
                   DriveEnableFunc enable_drive);

  auto Update() -> Progress;
  [[nodiscard]] auto state() const -> Progress;

 private:
  // --- State ---
  Progress state_ = Progress::kDisabled;

  // --- Injected dependencies ---
  ReadObjectFunc read_object_;
  InputReadFunc read_inputs_;
  DriveDisableFunc disable_drive_;
  DriveEnableFunc enable_drive_;

  // --- Constants ---
  static constexpr uint16_t kIdxModeOfOperation = 0x6060;
  static constexpr uint16_t kIdxHomingMethod = 0x6098;

  static constexpr uint32_t kModeHoming = 0x6;
  static constexpr uint32_t kModeHomingStart = 0xF6;
  static constexpr uint32_t kHomingMethodDefault = 0x1;
};

}  // namespace motion_profile::homing
