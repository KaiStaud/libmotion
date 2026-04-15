#include "TrapezoidalRamp.hpp"
#include "../LinearProfiles/MotionStrategy.hpp"

#include <utility>

// Konstruktor
TrapezoidalRamp::TrapezoidalRamp(std::unique_ptr<MotionStrategy> strategy) noexcept
    : strategy_(std::move(strategy)) {}

// Destruktor (hier notwendig wegen forward declaration)
TrapezoidalRamp::~TrapezoidalRamp() = default;

// Setter
void TrapezoidalRamp::set_strategy(std::unique_ptr<MotionStrategy> strategy) noexcept {
    strategy_ = std::move(strategy);
}

// Getter
const MotionStrategy* TrapezoidalRamp::get_strategy() const noexcept {
    return strategy_.get();
}

TrapezoidalRamp::~TrapezoidalRamp() = default;