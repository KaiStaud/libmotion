#pragma once
#include <memory>
#include <utility>

#include "../LinearProfiles/MotionStrategy.hpp"

namespace motion_profile {
class Context {
  std::unique_ptr<motion_profile::MotionStrategy> motion_strat_;

 public:
  void setStrategy(std::unique_ptr<motion_profile::MotionStrategy> strat) {
    strat = std::move(strat);
  }

  void getFrequency() {
    if (motion_strat_) {
      motion_strat_->getFrequency();
    }
  }
};
}  // namespace motion_profile
