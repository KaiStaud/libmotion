#include <cstdint>
#include <variant>

#include "../LinearProfiles/MotionStrategy.hpp"
#include "../TrapezoidalRamp/TrapezoidalRamp.hpp"
#include "../LinearProfiles/LinearProfile.hpp"
#include "../Homing/Homing.hpp"
namespace motion_manager {
    
    enum class MotionModes:uint8_t{
        kNone=0,
        kProfilePositionMode = 1,
        kProfileVelocityMode = 3,
        kTorqueProfileMode = 4,
        kHomingMode =6 ,
        kCylicPositionMode = 8,
        kCylicVelocityMode = 9,
        kCylicProfileMode = 10,
    };

    enum class ProfileParameterized:uint8_t{
        kNoParameters = 0,
        kParameterized = 1,
        kParametersInvalid = 2,
    };

    class MotionManager{
        public:
        using InputReadFunc = bool (*)();
        using DriveEnableFunc = void (*)();
        using DriveDisableFunc = void (*)();
        using ReadObjectFunc = uint32_t (*)(uint16_t index, uint8_t subindex);

        MotionManager(ReadObjectFunc read_object,
                       InputReadFunc read_gpio,
                       DriveDisableFunc disable_drive,
                       DriveEnableFunc enable_drive);

        auto Initialize(motion_profile::TargetConstraints linear_constraints) -> void;
        auto RequestMode(uint8_t new_mode) -> MotionModes;
        auto GetFrequency(int t) -> double;
        using Mode = std::variant<motion_profile::LinearProfile,
        motion_profile::TrapezoidalRamp,
        motion_profile::ConstantVelocityProfile,
        motion_profile::homing::HomingController>;

        private:
        // --- Injected dependencies ---
        ReadObjectFunc read_object_;
        InputReadFunc read_inputs_;
        DriveDisableFunc disable_drive_;
        DriveEnableFunc enable_drive_;
        Mode mode_impl_;
        MotionModes mode_ = MotionModes::kNone;     
        ProfileParameterized initialized_ =  ProfileParameterized::kNoParameters;  
    };
}