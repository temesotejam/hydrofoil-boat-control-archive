#pragma once
#include <cmath>
#include <stdint.h>

namespace production_control {
inline bool motorRelayRequired(float duty) {
  return std::isfinite(duty) && std::fabs(duty) > 0.0f;
}

// negative/center/positive are semantic endpoints.  Their numeric order is
// deliberately unrestricted so mechanically reversed servos calibrate cleanly.
struct ServoTuning {
  float negativeUs, centerUs, positiveUs;

  constexpr ServoTuning(float negative = 1480.0f, float center = 1500.0f, float positive = 1520.0f)
      : negativeUs(negative), centerUs(center), positiveUs(positive) {
  }
};

struct ServoResult {
  uint16_t pulseUs = 1500;
  bool clamped = false, finite = true;
};

class ServoMapper {
public:
  explicit ServoMapper(const ServoTuning &tuning = ServoTuning{});
  void reset();
  ServoResult map(float normalized);
  bool setTuning(const ServoTuning &tuning);

  const ServoTuning &tuning() const {
    return tuning_;
  }

  bool validConfiguration() const {
    return valid(tuning_);
  }

  uint16_t previousPulseUs() const {
    return previousUs_;
  }

  static bool valid(const ServoTuning &tuning);

private:
  ServoTuning tuning_{};
  uint16_t previousUs_ = 1500;
};

class ServoSpikeFilter {
public:
  explicit ServoSpikeFilter(float threshold = .10f);
  void reset(float accepted = 0.0f);
  float apply(float desired);

  float accepted() const {
    return accepted_;
  }

private:
  float threshold_, accepted_ = 0.0f;
  int8_t pendingDirection_ = 0;
};

class DutyRamp {
public:
  constexpr DutyRamp(float maximum = .60f, float riseSeconds = 2.0f, float fallSeconds = .35f)
      : maximum_(maximum), riseSeconds_(riseSeconds), fallSeconds_(fallSeconds) {
  }

  void setMaximum(float maximum);
  void setRiseSeconds(float seconds);
  void setTarget(float duty);
  float step(float dtSeconds);

  void stopImmediate() {
    target_ = applied_ = 0.0f;
  }

  float target() const {
    return target_;
  }

  float applied() const {
    return applied_;
  }

  float maximum() const {
    return maximum_;
  }

  bool active() const {
    return target_ != applied_;
  }

private:
  float maximum_, riseSeconds_, fallSeconds_, target_ = 0.0f, applied_ = 0.0f;
};
}
