#pragma once

#include <memory>

// Forward Declaration → reduziert Compile-Abhängigkeiten
class MotionStrategy;

class TrapezoidalRamp {
public:
    // Konstruktor (explizit + ownership transfer)
    explicit TrapezoidalRamp(std::unique_ptr<MotionStrategy> strategy = nullptr) noexcept;

    // Rule of 5 (unique_ptr → kein Copy, nur Move)
    TrapezoidalRamp(const TrapezoidalRamp&) = delete;
    TrapezoidalRamp& operator=(const TrapezoidalRamp&) = delete;
    TrapezoidalRamp(TrapezoidalRamp&&) noexcept = default;
    TrapezoidalRamp& operator=(TrapezoidalRamp&&) noexcept = default;

    ~TrapezoidalRamp(); // out-of-line wegen forward declaration

    // Strategy setzen
    void set_strategy(std::unique_ptr<MotionStrategy> strategy) noexcept;

    // Zugriff (optional)
    [[nodiscard]] const MotionStrategy* get_strategy() const noexcept;

private:
    std::unique_ptr<MotionStrategy> strategy_;
};