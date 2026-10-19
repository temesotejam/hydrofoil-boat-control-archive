#include "production_control.h"

#include <cmath>
#include <cstring>

namespace production_control {
namespace {
constexpr float kPi = 3.14159265358979323846f;

float clampf(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}

float wrap(float value) {
  while (value > kPi)
    value -= 2 * kPi;
  while (value < -kPi)
    value += 2 * kPi;
  return value;
}

float mod2pi(float value) {
  value = fmodf(value, 2 * kPi);
  return value < 0 ? value + 2 * kPi : value;
}

struct Vec {
  float x, y;
}; // x=north, y=east: heading is atan2(y,x).

Vec add(Vec a, Vec b) {
  return {a.x + b.x, a.y + b.y};
}

Vec sub(Vec a, Vec b) {
  return {a.x - b.x, a.y - b.y};
}

Vec mul(Vec a, float s) {
  return {a.x * s, a.y * s};
}

float length(Vec a) {
  return sqrtf(a.x * a.x + a.y * a.y);
}

Vec unit(Vec a) {
  const float l = length(a);
  return l > .0001f ? mul(a, 1.0f / l) : Vec{0, 0};
}

float angle(Vec a) {
  return atan2f(a.y, a.x);
}

struct Pose {
  float n, e, heading;
};

struct DubinsPlan {
  float p[3]{};
  float total = 0;
  char type[4]{};
  bool valid = false;
};

bool dubinsParams(float alpha, float beta, float d, const char *type, float out[3]) {
  const float sa = sinf(alpha), sb = sinf(beta), ca = cosf(alpha), cb = cosf(beta),
              cab = cosf(alpha - beta);
  float tmp, p2, p;
  if (!strcmp(type, "LSL")) {
    tmp = d + sa - sb;
    p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb);
    if (p2 < 0)
      return false;
    tmp = atan2f(cb - ca, tmp);
    out[0] = mod2pi(-alpha + tmp);
    out[1] = sqrtf(p2);
    out[2] = mod2pi(beta - tmp);
    return true;
  }
  if (!strcmp(type, "RSR")) {
    tmp = d - sa + sb;
    p2 = 2 + d * d - 2 * cab + 2 * d * (-sa + sb);
    if (p2 < 0)
      return false;
    tmp = atan2f(ca - cb, tmp);
    out[0] = mod2pi(alpha - tmp);
    out[1] = sqrtf(p2);
    out[2] = mod2pi(-beta + tmp);
    return true;
  }
  if (!strcmp(type, "LSR")) {
    p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb);
    if (p2 < 0)
      return false;
    p = sqrtf(p2);
    tmp = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0f, p);
    out[0] = mod2pi(-alpha + tmp);
    out[1] = p;
    out[2] = mod2pi(-beta + tmp);
    return true;
  }
  if (!strcmp(type, "RSL")) {
    p2 = -2 + d * d + 2 * cab - 2 * d * (sa + sb);
    if (p2 < 0)
      return false;
    p = sqrtf(p2);
    tmp = atan2f(ca + cb, d - sa - sb) - atan2f(2.0f, p);
    out[0] = mod2pi(alpha - tmp);
    out[1] = p;
    out[2] = mod2pi(beta - tmp);
    return true;
  }
  if (!strcmp(type, "RLR")) {
    tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8;
    if (fabsf(tmp) > 1)
      return false;
    p = mod2pi(2 * kPi - acosf(tmp));
    const float t = mod2pi(alpha - atan2f(ca - cb, d - sa + sb) + p / 2);
    out[0] = t;
    out[1] = p;
    out[2] = mod2pi(alpha - beta - t + p);
    return true;
  }
  if (!strcmp(type, "LRL")) {
    tmp = (6 - d * d + 2 * cab + 2 * d * (-sa + sb)) / 8;
    if (fabsf(tmp) > 1)
      return false;
    p = mod2pi(2 * kPi - acosf(tmp));
    const float t = mod2pi(-alpha - atan2f(ca - cb, d + sa - sb) + p / 2);
    out[0] = t;
    out[1] = p;
    out[2] = mod2pi(beta - alpha - t + p);
    return true;
  }
  return false;
}

DubinsPlan dubinsPlan(const Pose &start, const Pose &target, float radius) {
  const float dx = (target.n - start.n) / radius, dy = (target.e - start.e) / radius,
              c = cosf(start.heading), s = sinf(start.heading);
  const float x = c * dx + s * dy, y = -s * dx + c * dy, d = sqrtf(x * x + y * y),
              th = atan2f(y, x), alpha = mod2pi(-th),
              beta = mod2pi(wrap(target.heading - start.heading) - th);
  const char *types[] = {"LSL", "RSR", "LSR", "RSL", "RLR", "LRL"};
  DubinsPlan best{};
  for (const char *type : types) {
    float p[3]{};
    if (!dubinsParams(alpha, beta, d, type, p))
      continue;
    const float total = (p[0] + p[1] + p[2]) * radius;
    if (!best.valid || total < best.total) {
      best.valid = true;
      best.total = total;
      memcpy(best.p, p, sizeof(p));
      strcpy(best.type, type);
    }
  }
  return best;
}

Pose poseOnDubins(const Pose &start, const DubinsPlan &plan, float distance) {
  float n = 0, e = 0, h = 0;
  // The caller supplies distance in metres; reconstruct radius from plan.total / normalised length.
  const float units = plan.p[0] + plan.p[1] + plan.p[2];
  const float radius = units > .0001f ? plan.total / units : 1.0f;
  float remain = distance / radius;
  for (uint8_t i = 0; i < 3 && remain > 0; ++i) {
    const float take = fminf(remain, plan.p[i]);
    const char m = plan.type[i];
    if (m == 'S') {
      n += take * cosf(h);
      e += take * sinf(h);
    } else if (m == 'L') {
      n += sinf(h + take) - sinf(h);
      e += -cosf(h + take) + cosf(h);
      h = wrap(h + take);
    } else {
      n += sinf(h) - sinf(h - take);
      e += cosf(h - take) - cosf(h);
      h = wrap(h - take);
    }
    remain -= take;
  }
  const float c = cosf(start.heading), s = sinf(start.heading);
  return Pose{start.n + radius * (c * n - s * e),
              start.e + radius * (s * n + c * e),
              wrap(start.heading + h)};
}
}

Controller::Controller(const Config &config) : config_(config) {
  reset();
}

void Controller::setConfig(const Config &config) {
  config_ = config;
  reset();
}

void Controller::reset() {
  mode_ = ControlMode::Manual;
  manual_ = {};
  manualReceivedUs_ = 0;
  stallStartUs_ = 0;
  targetYaw_ = 0;
  headingSet_ = false;
  segmentStartSet_ = false;
  waypointCount_ = activeWaypoint_ = 0;
  segmentStartNorth_ = segmentStartEast_ = 0;
  competitionCourse_ = recovering_ = false;
  competitionGateCount_ = activeCompetitionGate_ = recoveryGateCount_ = activeRecoveryGate_ =
      recoveryRejoinGate_ = 0;
  lastPositionValid_ = lastGnssValid_ = false;
  lastGnssUs_ = recoveryCrossTrackStartUs_ = 0;
  driftNorthMps_ = driftEastMps_ = lastCrossTrackM_ = 0;
  resetCommandFilters();
  for (float &value : previous_)
    value = 0;
}

bool Controller::finite(float value) const {
  return std::isfinite(value);
}

void Controller::resetCommandFilters() {
  pitchCommandFiltered_ = rollCommandFiltered_ = heightCommandFiltered_ = yawCommandFiltered_ = 0;
  pitchCommandFilterUs_ = rollCommandFilterUs_ = heightCommandFilterUs_ = yawCommandFilterUs_ = 0;
}

float Controller::filterCommand(
    float input, float cutoff, uint64_t nowUs, float &filtered, uint64_t &filteredUs) {
  if (!finite(input)) {
    filtered = 0;
    filteredUs = 0;
    return 0;
  }
  if (cutoff <= 0) {
    filtered = input;
    filteredUs = nowUs;
    return input;
  }
  float dt = .02f;
  if (filteredUs && nowUs > filteredUs) {
    dt = static_cast<float>(nowUs - filteredUs) * 1e-6f;
  }
  if (dt > .25f)
    dt = .02f;
  const float alpha = 1.0f - expf(-2.0f * kPi * cutoff * dt);
  filtered += clampf(alpha, 0.0f, 1.0f) * (input - filtered);
  filteredUs = nowUs;
  return filtered;
}

bool Controller::fresh(bool valid, uint64_t timestamp, uint64_t now, uint32_t maxAge) const {
  return valid && timestamp && now >= timestamp && now - timestamp <= maxAge;
}

CommandResult Controller::setMode(ControlMode mode, uint32_t, AuthoritativeSafety safety) {
  if (safety != AuthoritativeSafety::Disarmed)
    return {Ack::Rejected, 1};
  if (static_cast<uint8_t>(mode) > static_cast<uint8_t>(ControlMode::WaypointOnly))
    return {Ack::Rejected, 2};
  mode_ = mode;
  headingSet_ = false;
  stallStartUs_ = 0;
  return {Ack::Accepted, 0};
}

CommandResult Controller::setManual(const ManualCommand &command, uint64_t receivedUs) {
  if (!finite(command.leftFront) || !finite(command.rightFront) || !finite(command.rearYaw) ||
      !finite(command.propulsion))
    return {Ack::Rejected, 3};
  if (command.leftFront < -1 || command.leftFront > 1 || command.rightFront < -1 ||
      command.rightFront > 1 || command.rearYaw < -1 || command.rearYaw > 1 ||
      command.propulsion < 0 || command.propulsion > 1)
    return {Ack::Rejected, 4};
  if (command.enabledMask & ~ManualAll)
    return {Ack::Rejected, 4};
  manual_ = command;
  manualReceivedUs_ = receivedUs;
  return {Ack::Accepted, 0};
}

CommandResult Controller::setHeading(float yawRad, uint32_t) {
  if (!finite(yawRad))
    return {Ack::Rejected, 3};
  targetYaw_ = wrap(yawRad);
  headingSet_ = true;
  return {Ack::Accepted, 0};
}

CommandResult Controller::setWaypoints(const Waypoint *points,
                                       uint8_t count,
                                       uint32_t,
                                       AuthoritativeSafety safety,
                                       bool competitionCourse) {
  if (safety != AuthoritativeSafety::Disarmed || !points || !count || count > 16)
    return {Ack::Rejected, 5};
  for (uint8_t i = 0; i < count; ++i)
    if (!finite(points[i].northM) || !finite(points[i].eastM))
      return {Ack::Rejected, 3};
  for (uint8_t i = 0; i < count; ++i)
    waypoints_[i] = points[i];
  waypointCount_ = count;
  activeWaypoint_ = 0;
  competitionCourse_ = competitionCourse;
  buildCompetitionGates();
  resetNavigationState();
  return {Ack::Accepted, 0};
}

void Controller::resetNavigationState() {
  activeWaypoint_ = 0;
  activeCompetitionGate_ = 0;
  recoveryGateCount_ = activeRecoveryGate_ = recoveryRejoinGate_ = 0;
  recovering_ = false;
  segmentStartSet_ = false;
  lastPositionValid_ = lastGnssValid_ = false;
  lastGnssUs_ = recoveryCrossTrackStartUs_ = 0;
  driftNorthMps_ = driftEastMps_ = lastCrossTrackM_ = 0;
}

CommandResult Controller::setWaypointReachRadius(float radiusM, AuthoritativeSafety safety) {
  if (safety != AuthoritativeSafety::Disarmed || !finite(radiusM) || radiusM < .5f ||
      radiusM > 20.0f)
    return {Ack::Rejected, 6};
  config_.waypointReachM = radiusM;
  return {Ack::Accepted, 0};
}

void Controller::buildCompetitionGates() {
  competitionGateCount_ = 0;
  if (!competitionCourse_ || waypointCount_ != 11)
    return;
  const float radius = config_.competitionMinTurnRadiusM * config_.competitionGateRadiusFactor;
  const float clearance = 4.0f;
  constexpr uint8_t arcCount = 15, alignCount = 4;
  auto addGate =
      [this](float north, float east, float heading, uint8_t routeIndex, bool goal = false) {
        if (competitionGateCount_ >= kMaxCompetitionGates)
          return;
        CompetitionGate &gate = competitionGates_[competitionGateCount_++];
        gate.point.northM = north;
        gate.point.eastM = east;
        gate.headingRad = wrap(heading);
        gate.routeIndex = routeIndex;
        gate.goal = goal;
      };

  // The source simulator uses x=east/y=north.  Build in that frame, then
  // convert headings back to this controller's north/east convention.
  struct SimVec {
    float x, y;
  };

  auto subV = [](SimVec a, SimVec b) {
    return SimVec{a.x - b.x, a.y - b.y};
  };
  auto addV = [](SimVec a, SimVec b) {
    return SimVec{a.x + b.x, a.y + b.y};
  };
  auto mulV = [](SimVec a, float s) {
    return SimVec{a.x * s, a.y * s};
  };
  auto lenV = [](SimVec a) {
    return sqrtf(a.x * a.x + a.y * a.y);
  };
  auto normV = [&](SimVec a) {
    const float l = lenV(a);
    return l > .001f ? mulV(a, 1.0f / l) : SimVec{0, 0};
  };
  auto angleV = [](SimVec a) {
    return atan2f(a.y, a.x);
  };
  for (uint8_t i = 1; i + 1 < waypointCount_; ++i) {
    const SimVec prev{waypoints_[i - 1].eastM, waypoints_[i - 1].northM},
        p{waypoints_[i].eastM, waypoints_[i].northM},
        next{waypoints_[i + 1].eastM, waypoints_[i + 1].northM};
    const SimVec vin = normV(subV(p, prev)), vout = normV(subV(next, p));
    SimVec b = addV(normV(subV(prev, p)), normV(subV(next, p)));
    if (lenV(b) < .001f) {
      const float cross = vin.x * vout.y - vin.y * vout.x;
      b = {-vout.y * (cross >= 0 ? 1.0f : -1.0f), vout.x * (cross >= 0 ? 1.0f : -1.0f)};
    }
    const SimVec n = mulV(normV(b), -1.0f), contact = addV(p, mulV(n, clearance)),
                 center = addV(p, mulV(n, clearance - radius));
    const float centerAngle = angleV(subV(contact, center));
    SimVec mean = normV(addV(vin, vout));
    if (lenV(mean) < .001f)
      mean = vout;
    const float tangPlus = atan2f(cosf(centerAngle), -sinf(centerAngle)); // (-sin, cos)
    const int orient =
        fabsf(wrap(tangPlus - angleV(mean))) <= fabsf(wrap(tangPlus + kPi - angleV(mean))) ? 1 : -1;
    const float thetaIn = angleV(vin) - orient * kPi * .5f,
                thetaOut = angleV(vout) - orient * kPi * .5f;
    float sIn = orient * wrap(thetaIn - centerAngle), sOut = orient * wrap(thetaOut - centerAngle);
    if (sIn > 0)
      sIn -= 2 * kPi;
    if (sOut < 0)
      sOut += 2 * kPi;
    const float minHalf = kPi / 30.0f;
    if (fabsf(sIn) < minHalf)
      sIn = -minHalf;
    if (fabsf(sOut) < minHalf)
      sOut = minHalf;
    SimVec last{0, 0};
    float lastHeading = 0;
    for (uint8_t k = 0; k < arcCount; ++k) {
      const float q = static_cast<float>(k) / (arcCount - 1);
      const float sweep = q <= .5f ? sIn * (1.0f - q / .5f) : sOut * ((q - .5f) / .5f);
      const float theta = centerAngle + orient * sweep;
      const SimVec now{center.x + radius * cosf(theta), center.y + radius * sinf(theta)};
      if (k) {
        const SimVec d = subV(now, last);
        lastHeading = wrap(kPi * .5f - angleV(d));
      }
      last = now;
      addGate(now.y, now.x, lastHeading, i);
    }
    // Update the previous arc point heading now that its successor is known.
    const uint8_t first = competitionGateCount_ - arcCount;
    for (uint8_t k = 0; k + 1 < arcCount; ++k) {
      const auto &a = competitionGates_[first + k].point;
      const auto &bpt = competitionGates_[first + k + 1].point;
      competitionGates_[first + k].headingRad = atan2f(bpt.eastM - a.eastM, bpt.northM - a.northM);
    }
    if (i + 2 < waypointCount_) {
      // Same cubic-Hermite alignment gates as the simulator, joining this arc
      // to the next buoy arc once its start point is constructed below.
      // They are inserted on the next pass after both endpoints are known.
      const uint8_t nextIndex = i + 1;
      const SimVec p2{waypoints_[nextIndex].eastM, waypoints_[nextIndex].northM};
      const SimVec toward = normV(subV(p2, last));
      for (uint8_t k = 1; k <= alignCount; ++k) {
        const float t = static_cast<float>(k) / (alignCount + 1);
        const float d = 160.0f * t;
        const SimVec q = addV(last, mulV(toward, d));
        addGate(q.y, q.x, wrap(kPi * .5f - angleV(toward)), i);
      }
    }
  }
  const Waypoint goal = waypoints_[waypointCount_ - 1];
  const CompetitionGate &prior = competitionGates_[competitionGateCount_ - 1];
  addGate(goal.northM,
          goal.eastM,
          atan2f(goal.eastM - prior.point.eastM, goal.northM - prior.point.northM),
          waypointCount_ - 1,
          true);
}

void Controller::updateDriftEstimate(const SensorInput &input) {
  if (!competitionCourse_ || !input.gnssUs || input.gnssUs == lastGnssUs_)
    return;
  if (lastGnssValid_ && input.gnssUs > lastGnssUs_) {
    const float dt = static_cast<float>(input.gnssUs - lastGnssUs_) * 1e-6f;
    if (dt > .05f && dt < 3.0f) {
      const float groundN = (input.northM - lastGnssNorth_) / dt,
                  groundE = (input.eastM - lastGnssEast_) / dt;
      const float ownN = config_.competitionCruiseSpeedMps * cosf(input.yawRad),
                  ownE = config_.competitionCruiseSpeedMps * sinf(input.yawRad);
      float measureN = groundN - ownN, measureE = groundE - ownE;
      const float m = sqrtf(measureN * measureN + measureE * measureE);
      if (m > config_.competitionDriftMaxMps) {
        measureN *= config_.competitionDriftMaxMps / m;
        measureE *= config_.competitionDriftMaxMps / m;
      }
      const float a = 1.0f - expf(-dt / config_.competitionDriftTauS);
      driftNorthMps_ += a * (measureN - driftNorthMps_);
      driftEastMps_ += a * (measureE - driftEastMps_);
    }
  }
  lastGnssNorth_ = input.northM;
  lastGnssEast_ = input.eastM;
  lastGnssUs_ = input.gnssUs;
  lastGnssValid_ = true;
}

bool Controller::competitionGatePassed(const SensorInput &input, const Output &output) const {
  const CompetitionGate &gate =
      recovering_ ? recoveryGates_[activeRecoveryGate_] : competitionGates_[activeCompetitionGate_];
  const float near = gate.goal ? config_.waypointReachM : config_.competitionGateWidthM * 1.15f;
  if (output.waypointDistanceM <= near)
    return true;
  if (!lastPositionValid_)
    return false;
  const float pn = lastPositionNorth_ - gate.point.northM,
              pe = lastPositionEast_ - gate.point.eastM, cn = input.northM - gate.point.northM,
              ce = input.eastM - gate.point.eastM;
  const float alongPrev = pn * cosf(gate.headingRad) + pe * sinf(gate.headingRad),
              alongNow = cn * cosf(gate.headingRad) + ce * sinf(gate.headingRad);
  const float lateral = -sinf(gate.headingRad) * cn + cosf(gate.headingRad) * ce;
  return alongPrev <= 0 && alongNow >= 0 && fabsf(lateral) < config_.competitionGateWidthM;
}

bool Controller::startDubinsRecovery(const SensorInput &input) {
  if (!competitionCourse_ || recovering_ || !competitionGateCount_)
    return false;
  const uint8_t end =
      static_cast<uint8_t>(fminf(competitionGateCount_ - 1, activeCompetitionGate_ + 14));
  uint8_t best = activeCompetitionGate_;
  float bestScore = INFINITY;
  const Pose start{input.northM, input.eastM, input.yawRad};
  for (uint8_t i = activeCompetitionGate_; i <= end; ++i) {
    const auto &g = competitionGates_[i];
    const Pose targetPose{g.point.northM, g.point.eastM, g.headingRad};
    const DubinsPlan plan = dubinsPlan(start, targetPose, config_.competitionMinTurnRadiusM);
    if (!plan.valid)
      continue;
    const float dn = g.point.northM - input.northM, de = g.point.eastM - input.eastM;
    const float score = plan.total + 40.0f * fabsf(wrap(atan2f(de, dn) - input.yawRad)) +
                        15.0f * (i - activeCompetitionGate_);
    if (score < bestScore) {
      bestScore = score;
      best = i;
    }
  }
  const auto &target = competitionGates_[best];
  const Pose targetPose{target.point.northM, target.point.eastM, target.headingRad};
  const DubinsPlan plan = dubinsPlan(start, targetPose, config_.competitionMinTurnRadiusM);
  if (!plan.valid)
    return false;
  recoveryGateCount_ =
      static_cast<uint8_t>(fminf(kMaxRecoveryGates, fmaxf(9.0f, ceilf(plan.total / 70.0f))));
  for (uint8_t i = 0; i < recoveryGateCount_; ++i) {
    const Pose p = poseOnDubins(
        start, plan, plan.total * static_cast<float>(i + 1) / (recoveryGateCount_ + 1));
    CompetitionGate &gate = recoveryGates_[i];
    gate.point.northM = p.n;
    gate.point.eastM = p.e;
    gate.headingRad = p.heading;
    gate.routeIndex = target.routeIndex;
    gate.goal = false;
  }
  activeRecoveryGate_ = 0;
  recoveryRejoinGate_ = best;
  recovering_ = true;
  activeWaypoint_ = target.routeIndex;
  segmentStartNorth_ = input.northM;
  segmentStartEast_ = input.eastM;
  segmentStartSet_ = true;
  return true;
}

bool Controller::physicalConfigurationValid(const PhysicalConfig &config) {
  const uint8_t channels[] = {
      config.leftChannel, config.rightChannel, config.rearChannel, config.propulsionChannel};
  for (uint8_t i = 0; i < 4; ++i) {
    if (channels[i] > 15)
      return false;
    for (uint8_t j = i + 1; j < 4; ++j)
      if (channels[i] == channels[j])
        return false;
  }
  return config.leftMinUs < config.leftCenterUs && config.leftCenterUs < config.leftMaxUs &&
         config.rightMinUs < config.rightCenterUs && config.rightCenterUs < config.rightMaxUs &&
         config.rearMinUs < config.rearCenterUs && config.rearCenterUs < config.rearMaxUs &&
         config.propMinUs < config.propStopUs && config.propStopUs < config.propMaxUs &&
         config.calibrationComplete;
}

float Controller::limit(float value, float previous, bool &saturated) const {
  const float bounded = clampf(value, -1.0f, 1.0f);
  if (bounded != value)
    saturated = true;
  value = bounded;
  const float delta = value - previous;
  if (delta > config_.slewPerStep) {
    value = previous + config_.slewPerStep;
    saturated = true;
  }
  if (delta < -config_.slewPerStep) {
    value = previous - config_.slewPerStep;
    saturated = true;
  }
  return value;
}

void Controller::safe(Output &output, StopReason reason, SafetyRequest request) const {
  output.leftFront = output.rightFront = output.rearYaw = output.propulsion = 0;
  output.reason = reason;
  output.safetyRequest = request;
  output.physicalGate = false;
}

float Controller::speedBlend(float speed) const {
  return clampf(
      (speed - config_.lowSpeedMps) / (config_.highSpeedMps - config_.lowSpeedMps), 0.0f, 1.0f);
}

float Controller::waypointCourse(const SensorInput &input, Output &output) {
  if (!segmentStartSet_) {
    segmentStartNorth_ = input.northM;
    segmentStartEast_ = input.eastM;
    segmentStartSet_ = true;
  }
  const Waypoint &target = competitionCourse_
                               ? (recovering_ ? recoveryGates_[activeRecoveryGate_].point
                                              : competitionGates_[activeCompetitionGate_].point)
                               : waypoints_[activeWaypoint_];
  const float dn = target.northM - input.northM, de = target.eastM - input.eastM;
  output.waypointDistanceM = std::sqrt(dn * dn + de * de);
  const float pathN = target.northM - segmentStartNorth_, pathE = target.eastM - segmentStartEast_;
  const float pathLength = std::sqrt(pathN * pathN + pathE * pathE);
  if (pathLength < .01f)
    return std::atan2(de, dn);
  const float pathCourse = std::atan2(pathE, pathN);
  const float relN = input.northM - segmentStartNorth_, relE = input.eastM - segmentStartEast_;
  const float crossTrack = -std::sin(pathCourse) * relN + std::cos(pathCourse) * relE;
  lastCrossTrackM_ = crossTrack;
  if (competitionCourse_) {
    updateDriftEstimate(input);
    float nearest = INFINITY;
    for (uint8_t i = 0; i < waypointCount_; ++i) {
      const float n = waypoints_[i].northM - input.northM, e = waypoints_[i].eastM - input.eastM;
      nearest = fminf(nearest, sqrtf(n * n + e * e));
    }
    const float q = clampf(nearest / config_.competitionNearRadiusM, 0.0f, 1.0f);
    float look = config_.competitionLookNearM * (1.0f - q) + config_.competitionLookFarM * q -
                 config_.competitionLosErrorReduce * fabsf(crossTrack);
    look = clampf(look,
                  fmaxf(5.0f, config_.competitionLookNearM * .45f),
                  fmaxf(config_.competitionLookFarM, config_.competitionLookNearM));
    const float tangentN = cosf(pathCourse), tangentE = sinf(pathCourse),
                normalN = -sinf(pathCourse), normalE = cosf(pathCourse);
    const float crossVel = config_.competitionCruiseSpeedMps * crossTrack / look;
    float desiredN =
        config_.competitionCruiseSpeedMps * tangentN - crossVel * normalN - driftNorthMps_;
    float desiredE =
        config_.competitionCruiseSpeedMps * tangentE - crossVel * normalE - driftEastMps_;
    if (fabsf(desiredN) + fabsf(desiredE) > .0001f)
      return atan2f(desiredE, desiredN);
  }
  return wrap(pathCourse - std::atan2(crossTrack, config_.losLookaheadM));
}

Output Controller::step(const SensorInput &input) {
  Output output{};
  output.safety = input.safety;
  output.mode = mode_;
  output.activeWaypoint = activeWaypoint_;
  output.competitionCourse = competitionCourse_;
  output.recovering = recovering_;
  if (input.safety != AuthoritativeSafety::Running) {
    resetCommandFilters();
    safe(output,
         input.safety == AuthoritativeSafety::EStop ? StopReason::EStop : StopReason::None,
         SafetyRequest::None);
    return output;
  }
  if (!fresh(input.heartbeat, input.heartbeatUs, input.nowUs, config_.heartbeatStaleUs)) {
    safe(output, StopReason::Heartbeat, SafetyRequest::Fault);
    return output;
  }
  const bool manualMode = mode_ == ControlMode::Manual;
  const bool waypointMode = modeUsesWaypoint(mode_);
  const bool attitudeControl = modeUsesAttitude(mode_);
  const bool needsManual = modeNeedsManual(mode_);
  if (needsManual && !fresh(true, manualReceivedUs_, input.nowUs, config_.manualStaleUs)) {
    safe(output, StopReason::ManualTimeout, SafetyRequest::Disarm);
    return output;
  }
  if (attitudeControl && !fresh(input.imuValid, input.imuUs, input.nowUs, config_.imuStaleUs)) {
    safe(output,
         input.imuValid ? StopReason::ImuStale : StopReason::ImuInvalid,
         SafetyRequest::Fault);
    return output;
  }
  const bool waypointNavigationReady =
      waypointMode && fresh(input.gnssValid, input.gnssUs, input.nowUs, config_.gnssStaleUs) &&
      waypointCount_;
  if (waypointMode && !waypointNavigationReady)
    output.reason = input.gnssValid ? StopReason::GnssStale : StopReason::GnssInvalid;
  // Disabled in the current human-monitored configuration. Keep the optional
  // trip behind an explicit setting so a later unattended operating profile
  // can restore it without changing controller logic.
  if (config_.enableAttitudeDangerTrip && input.imuValid &&
      (std::fabs(input.pitchRad) >= config_.attitudeStopRad ||
       std::fabs(input.rollRad) >= config_.attitudeStopRad)) {
    safe(output, StopReason::AttitudeDanger, SafetyRequest::Fault);
    return output;
  }

  if (manualMode) {
    output.enabledMask = manual_.enabledMask;
    output.leftPrelimit = (manual_.enabledMask & ManualLeft) ? manual_.leftFront : 0;
    output.rightPrelimit = (manual_.enabledMask & ManualRight) ? manual_.rightFront : 0;
    output.rearPrelimit = (manual_.enabledMask & ManualRear) ? manual_.rearYaw : 0;
    output.propulsionPrelimit = (manual_.enabledMask & ManualPropulsion) ? manual_.propulsion : 0;
  } else {
    output.enabledMask = ManualAll;
    if (attitudeControl) {
      output.uPitch = config_.kpPitch * (config_.targetPitch - input.pitchRad) -
                      config_.kdPitch * input.pitchRateRadS;
      output.uRoll = config_.kpRoll * (config_.targetRoll - input.rollRad) -
                     config_.kdRoll * input.rollRateRadS;
      const bool tofFresh = fresh(input.tofValid, input.tofUs, input.nowUs, config_.tofStaleUs);
      if (tofFresh)
        output.uHeight = config_.kpHeight * (config_.targetHeightM - input.tofM);
      else {
        heightCommandFiltered_ = 0;
        heightCommandFilterUs_ = 0;
        output.uHeight = 0;
        output.flags |= HeightDegraded;
        output.reason = input.tofValid ? StopReason::TofStale : StopReason::TofInvalid;
      }
      bool heightControlActive = tofFresh;
      if (std::fabs(input.pitchRad) >= config_.pitchPriorityRad) {
        output.uPitch = clampf(output.uPitch * 1.5f, -1.0f, 1.0f);
        heightControlActive = false;
        output.uHeight = 0;
        output.uRoll *= .25f;
        output.flags |= PitchPriority;
      }
      output.uPitch = filterCommand(output.uPitch,
                                    config_.pitchCommandCutoffHz,
                                    input.nowUs,
                                    pitchCommandFiltered_,
                                    pitchCommandFilterUs_);
      output.uRoll = filterCommand(output.uRoll,
                                   config_.rollCommandCutoffHz,
                                   input.nowUs,
                                   rollCommandFiltered_,
                                   rollCommandFilterUs_);
      if (heightControlActive)
        output.uHeight = filterCommand(output.uHeight,
                                       config_.heightCommandCutoffHz,
                                       input.nowUs,
                                       heightCommandFiltered_,
                                       heightCommandFilterUs_);
      else {
        heightCommandFiltered_ = 0;
        heightCommandFilterUs_ = 0;
      }
      const float common = output.uPitch + output.uHeight;
      output.leftPrelimit =
          clampf(common + output.uRoll, -config_.attitudeServoLimit, config_.attitudeServoLimit);
      output.rightPrelimit =
          clampf(common - output.uRoll, -config_.attitudeServoLimit, config_.attitudeServoLimit);
    } else {
      pitchCommandFiltered_ = rollCommandFiltered_ = heightCommandFiltered_ = 0;
      pitchCommandFilterUs_ = rollCommandFilterUs_ = heightCommandFilterUs_ = 0;
      // WaypointOnly keeps both front-wing servos actively at neutral while
      // removing roll, pitch, and ToF-height corrections.
      output.leftPrelimit = 0;
      output.rightPrelimit = 0;
    }
    if (waypointNavigationReady) {
      output.targetYaw = waypointCourse(input, output);
      const bool reached = competitionCourse_ ? competitionGatePassed(input, output)
                                              : output.waypointDistanceM <= config_.waypointReachM;
      if (reached) {
        output.waypointReached = true;
        if (competitionCourse_) {
          const CompetitionGate &passed = recovering_ ? recoveryGates_[activeRecoveryGate_]
                                                      : competitionGates_[activeCompetitionGate_];
          segmentStartNorth_ = passed.point.northM;
          segmentStartEast_ = passed.point.eastM;
          if (recovering_) {
            if (++activeRecoveryGate_ >= recoveryGateCount_) {
              recovering_ = false;
              activeCompetitionGate_ = recoveryRejoinGate_;
            }
          } else
            ++activeCompetitionGate_;
          if (!recovering_ && activeCompetitionGate_ >= competitionGateCount_) {
            safe(output, StopReason::FinalWaypoint, SafetyRequest::Disarm);
            return output;
          }
          if (!recovering_ && activeCompetitionGate_ >= competitionGateCount_) {
            safe(output, StopReason::FinalWaypoint, SafetyRequest::Disarm);
            return output;
          }
          activeWaypoint_ = recovering_ ? recoveryGates_[activeRecoveryGate_].routeIndex
                                        : competitionGates_[activeCompetitionGate_].routeIndex;
        } else {
          segmentStartNorth_ = waypoints_[activeWaypoint_].northM;
          segmentStartEast_ = waypoints_[activeWaypoint_].eastM;
          if (++activeWaypoint_ >= waypointCount_) {
            safe(output, StopReason::FinalWaypoint, SafetyRequest::Disarm);
            return output;
          }
        }
        output.activeWaypoint = activeWaypoint_;
        output.targetYaw = waypointCourse(input, output);
      }
      if (competitionCourse_ && !recovering_) {
        if (fabsf(lastCrossTrackM_) >= config_.competitionRecoveryCrossTrackM) {
          if (!recoveryCrossTrackStartUs_)
            recoveryCrossTrackStartUs_ = input.nowUs;
          if (input.nowUs - recoveryCrossTrackStartUs_ >= config_.competitionRecoveryHoldUs &&
              startDubinsRecovery(input)) {
            output.recovering = true;
            output.targetYaw = waypointCourse(input, output);
          }
        } else
          recoveryCrossTrackStartUs_ = 0;
      }
      lastPositionNorth_ = input.northM;
      lastPositionEast_ = input.eastM;
      lastPositionValid_ = true;
    } else if (waypointMode) {
      // In degraded bench operation, retain any available attitude control but
      // do not create a course command or propulsion without fresh GNSS.
      output.targetYaw = input.yawRad;
      output.propulsionPrelimit = 0;
    } else if (mode_ == ControlMode::HeadingHold) {
      if (!headingSet_) {
        targetYaw_ = wrap(input.yawRad);
        headingSet_ = true;
      }
      output.targetYaw = targetYaw_;
    }
    if (mode_ == ControlMode::HeadingHold || waypointMode) {
      output.courseErrorRad = wrap(output.targetYaw - input.yawRad);
      const float blend = speedBlend(input.groundSpeedMps);
      const float gain = 1.0f + blend * (config_.highSpeedYawGain - 1.0f);
      const float yawLimit = 1.0f + blend * (config_.highSpeedYawLimit - 1.0f);
      output.uYaw =
          clampf(gain * config_.kpYaw * output.courseErrorRad - config_.kdYaw * input.yawRateRadS,
                 -yawLimit,
                 yawLimit);
      output.uYaw = filterCommand(output.uYaw,
                                  config_.yawCommandCutoffHz,
                                  input.nowUs,
                                  yawCommandFiltered_,
                                  yawCommandFilterUs_);
      output.rearPrelimit = output.uYaw;
      output.flags |= YawScheduled;
    } else {
      yawCommandFiltered_ = 0;
      yawCommandFilterUs_ = 0;
      output.rearPrelimit = manual_.rearYaw;
    }
    output.propulsionPrelimit = waypointMode
                                    ? (waypointNavigationReady ? config_.autoPropulsion : 0.0f)
                                    : manual_.propulsion;
  }

  if (!finite(output.leftPrelimit) || !finite(output.rightPrelimit) ||
      !finite(output.rearPrelimit) || !finite(output.propulsionPrelimit)) {
    safe(output, StopReason::NonFinite, SafetyRequest::Fault);
    return output;
  }
  if (output.propulsionPrelimit > .02f) {
    const bool vescFresh = fresh(input.vescValid, input.vescUs, input.nowUs, config_.vescStaleUs);
    const bool powerFresh =
        fresh(input.powerValid, input.powerUs, input.nowUs, config_.powerStaleUs);
    if (input.vescFault || !vescFresh || !powerFresh) {
      const StopReason unavailable =
          input.vescFault
              ? StopReason::VescFault
              : (!vescFresh
                     ? StopReason::VescStale
                     : (input.powerValid ? StopReason::PowerStale : StopReason::PowerInvalid));
      output.propulsionPrelimit = 0;
      output.flags |= PropulsionUnavailable;
      output.reason = unavailable;
    }
  }
  if (output.propulsionPrelimit > .02f) {
    if (input.busVoltageV <= config_.criticalVoltageV) {
      safe(output, StopReason::LowVoltage, SafetyRequest::Fault);
      return output;
    }
    if (std::fabs(input.currentA) >= config_.criticalCurrentA) {
      safe(output, StopReason::OverCurrent, SafetyRequest::Fault);
      return output;
    }
    if (input.busVoltageV < config_.lowVoltageV) {
      output.throttleLimit = .50f;
      output.flags |= VoltageLimited;
      output.reason = StopReason::LowVoltage;
    }
    if (std::fabs(input.currentA) > config_.overCurrentA) {
      output.throttleLimit = clampf(output.throttleLimit, 0.0f, .30f);
      output.flags |= CurrentLimited;
      output.reason = StopReason::OverCurrent;
    }
    const bool stalled = output.propulsionPrelimit >= config_.stallCommand &&
                         std::fabs(input.vescErpm) < config_.stallErpm &&
                         std::fabs(input.currentA) > config_.stallCurrentA;
    if (stalled) {
      if (!stallStartUs_)
        stallStartUs_ = input.nowUs;
      if (input.nowUs - stallStartUs_ >= config_.stallTripUs) {
        safe(output, StopReason::MotorStall, SafetyRequest::Fault);
        return output;
      }
    } else
      stallStartUs_ = 0;
    const bool cavitating = std::fabs(input.vescErpm) > config_.cavitationErpm &&
                            std::fabs(input.currentA) < config_.cavitationCurrentA &&
                            input.groundSpeedMps < config_.cavitationSpeedMps;
    if (cavitating) {
      output.throttleLimit = clampf(output.throttleLimit, 0.0f, .50f);
      output.flags |= CavitationLimited;
      output.reason = StopReason::Cavitation;
    }
    if (output.flags & PitchPriority)
      output.throttleLimit = clampf(output.throttleLimit, 0.0f, .25f);
  } else
    stallStartUs_ = 0;

  output.propulsionPrelimit = clampf(output.propulsionPrelimit, 0.0f, output.throttleLimit);
  output.leftFront = clampf(output.leftPrelimit, -1.0f, 1.0f);
  output.rightFront = clampf(output.rightPrelimit, -1.0f, 1.0f);
  output.rearYaw = clampf(output.rearPrelimit, -1.0f, 1.0f);
  if (output.leftFront != output.leftPrelimit || output.rightFront != output.rightPrelimit ||
      output.rearYaw != output.rearPrelimit)
    output.saturated = true;
  output.propulsion = limit(output.propulsionPrelimit, previous_[3], output.saturated);
  if (output.propulsion < 0)
    output.propulsion = 0;
  previous_[0] = output.leftFront;
  previous_[1] = output.rightFront;
  previous_[2] = output.rearYaw;
  previous_[3] = output.propulsion;
  output.physicalGate = true;
  return output;
}

const char *safetyName(AuthoritativeSafety value) {
  switch (value) {
  case AuthoritativeSafety::Boot:
    return "BOOT";
  case AuthoritativeSafety::Disarmed:
    return "DISARMED";
  case AuthoritativeSafety::ArmedIdle:
    return "ARMED_IDLE";
  case AuthoritativeSafety::Running:
    return "RUNNING";
  case AuthoritativeSafety::EStop:
    return "E_STOP";
  default:
    return "FAULT";
  }
}

const char *modeName(ControlMode value) {
  switch (value) {
  case ControlMode::Manual:
    return "MANUAL";
  case ControlMode::AttitudeAssist:
    return "ATTITUDE_ASSIST";
  case ControlMode::HeadingHold:
    return "HEADING_HOLD";
  case ControlMode::AutoWaypoint:
    return "AUTO_WAYPOINT";
  default:
    return "WAYPOINT_ONLY";
  }
}

const char *reasonName(StopReason value) {
  switch (value) {
  case StopReason::None:
    return "NONE";
  case StopReason::Stop:
    return "STOP";
  case StopReason::EStop:
    return "E_STOP";
  case StopReason::Heartbeat:
    return "HEARTBEAT";
  case StopReason::ManualTimeout:
    return "MANUAL_TIMEOUT";
  case StopReason::ImuInvalid:
    return "IMU_INVALID";
  case StopReason::ImuStale:
    return "IMU_STALE";
  case StopReason::TofInvalid:
    return "TOF_INVALID";
  case StopReason::TofStale:
    return "TOF_STALE";
  case StopReason::GnssInvalid:
    return "GNSS_INVALID";
  case StopReason::GnssStale:
    return "GNSS_STALE";
  case StopReason::VescFault:
    return "VESC_FAULT";
  case StopReason::NonFinite:
    return "NONFINITE";
  case StopReason::FinalWaypoint:
    return "FINAL_WAYPOINT";
  case StopReason::PowerInvalid:
    return "POWER_INVALID";
  case StopReason::PowerStale:
    return "POWER_STALE";
  case StopReason::LowVoltage:
    return "LOW_VOLTAGE";
  case StopReason::OverCurrent:
    return "OVER_CURRENT";
  case StopReason::MotorStall:
    return "MOTOR_STALL";
  case StopReason::AttitudeDanger:
    return "ATTITUDE_DANGER";
  case StopReason::VescStale:
    return "VESC_STALE";
  case StopReason::Cavitation:
    return "CAVITATION";
  default:
    return "GNSS_HEADING_TIMEOUT";
  }
}

} // namespace production_control
