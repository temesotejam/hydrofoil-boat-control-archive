#include "competition_actuators.h"
#include <cmath>

namespace production_control {
namespace {
float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
}

ServoMapper::ServoMapper(const ServoTuning &t) : tuning_(t) {
  reset();
}

bool ServoMapper::valid(const ServoTuning &t) {
  return std::isfinite(t.negativeUs) && std::isfinite(t.centerUs) && std::isfinite(t.positiveUs) &&
         t.negativeUs >= 500.0f && t.negativeUs <= 2500.0f && t.centerUs >= 500.0f &&
         t.centerUs <= 2500.0f && t.positiveUs >= 500.0f && t.positiveUs <= 2500.0f;
}

bool ServoMapper::setTuning(const ServoTuning &t) {
  if (!valid(t))
    return false;
  tuning_ = t;
  reset();
  return true;
}

void ServoMapper::reset() {
  previousUs_ = valid(tuning_) ? static_cast<uint16_t>(tuning_.centerUs + .5f) : 1500;
}

ServoResult ServoMapper::map(float normalized) {
  ServoResult r{};
  r.finite = std::isfinite(normalized) && valid(tuning_);
  if (!r.finite) {
    r.pulseUs = previousUs_;
    r.clamped = true;
    return r;
  }
  const float constrained = clampf(normalized, -1.0f, 1.0f);
  r.clamped = constrained != normalized;
  const float target =
      constrained >= 0
          ? tuning_.centerUs + constrained * (tuning_.positiveUs - tuning_.centerUs)
          : tuning_.centerUs + (-constrained) * (tuning_.negativeUs - tuning_.centerUs);
  previousUs_ = static_cast<uint16_t>(target + .5f);
  r.pulseUs = previousUs_;
  return r;
}

ServoSpikeFilter::ServoSpikeFilter(float threshold)
    : threshold_(std::isfinite(threshold) && threshold > 0 ? threshold : .10f) {
  reset();
}

void ServoSpikeFilter::reset(float accepted) {
  accepted_ = std::isfinite(accepted) ? clampf(accepted, -1.0f, 1.0f) : 0.0f;
  pendingDirection_ = 0;
}

float ServoSpikeFilter::apply(float desired) {
  if (!std::isfinite(desired))
    return desired;
  desired = clampf(desired, -1.0f, 1.0f);
  const float delta = desired - accepted_;
  if (std::fabs(delta) <= threshold_) {
    accepted_ = desired;
    pendingDirection_ = 0;
    return accepted_;
  }
  const int8_t direction = delta > 0 ? 1 : -1;
  if (pendingDirection_ != direction) {
    pendingDirection_ = direction;
    return accepted_;
  }
  accepted_ = desired;
  pendingDirection_ = 0;
  return accepted_;
}

void DutyRamp::setMaximum(float maximum) {
  maximum_ = std::isfinite(maximum) && maximum >= 0.0f ? maximum : 0.0f;
  target_ = clampf(target_, 0.0f, maximum_);
  applied_ = clampf(applied_, 0.0f, maximum_);
}

void DutyRamp::setRiseSeconds(float seconds) {
  if (std::isfinite(seconds) && seconds >= .10f && seconds <= 20.0f)
    riseSeconds_ = seconds;
}

void DutyRamp::setTarget(float duty) {
  target_ = std::isfinite(duty) ? clampf(duty, 0.0f, maximum_) : 0.0f;
}

float DutyRamp::step(float dt) {
  if (!std::isfinite(dt) || dt <= 0)
    return applied_;
  dt = clampf(dt, 0.0f, .10f);
  const float seconds = target_ > applied_ ? riseSeconds_ : fallSeconds_;
  const float maxDelta = seconds > 0 ? maximum_ * dt / seconds : maximum_;
  applied_ = clampf(target_, applied_ - maxDelta, applied_ + maxDelta);
  return applied_;
}
}
