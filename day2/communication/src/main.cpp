#include <Arduino.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_system.h>
#include <esp_timer.h>

#include <math.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "app_config.h"
#include "fixed_waypoints.h"
#include "gnss_receiver.h"
#include "production_page_ja.h"
#include "track_page_ja.h"
#include "diagnostics_page_ja.h"
#include "sensors_page_ja.h"
#include <boat_protocol.h>

using namespace app_config;

namespace {

constexpr uint32_t kLinkFreshMs = 1000;
constexpr uint32_t kHeartbeatPeriodMs = 100;
constexpr uint32_t kManualRefreshMs = 200;
constexpr uint32_t kCommandRetryMs = 100;
constexpr uint32_t kCommandTimeoutMs = 1200;
constexpr uint32_t kSafetyRetryMs = 150;
constexpr uint32_t kSafetyTimeoutMs = 2000;
constexpr uint32_t kStatusLogPeriodMs = 1000;
constexpr uint32_t kGnssFixFreshMs = 1500;
constexpr float kWaypointReachRadiusM = 1.5f;
constexpr size_t kRxChunkBytes = 512;

enum class Stage : uint8_t {
  Idle,
  EnsureDisarmed,
  WaitTuningAck,
  WaitWaypointAck,
  WaitModeAck,
  WaitManualAck,
  WaitArmed,
  WaitRunning,
  Running,
  Stopping,
  Emergency,
  ClearingEmergency,
  Error,
};

struct LinkCache {
  boat::ControlSnapshotPayload snapshot{};
  boat::ControlOutputPayload output{};
  boat::ActuatorStatePayload actuators{};
  boat::SystemHealthPayload health{};
  boat::InaStatusPayload ina{};
  boat::VescTelemetryPayload vesc{};
  boat::ControlCommandAckPayload commandAck{};
  boat::WaypointAckPayload waypointAck{};
  uint64_t lastFrameUs = 0;
  uint64_t lastSnapshotUs = 0;
  uint64_t lastOutputUs = 0;
  uint64_t lastActuatorUs = 0;
  uint64_t lastInaStatusUs = 0;
  uint64_t lastVescTelemetryUs = 0;
  uint64_t lastAckUs = 0;
  uint64_t lastWaypointAckUs = 0;
  uint32_t frames = 0;
  uint8_t latchedStopReason = 0;
  bool hasSnapshot = false;
  bool hasOutput = false;
  bool hasActuators = false;
  bool hasHealth = false;
  bool hasIna = false;
  bool hasVesc = false;
  bool hasCommandAck = false;
  bool hasWaypointAck = false;
  bool sawFinalWaypoint = false;
};

struct PendingCommand {
  boat::Type type = boat::Type::ControlModeCommand;
  uint8_t payload[sizeof(boat::WaypointSetPayload)]{};
  uint16_t length = 0;
  uint32_t requestId = 0;
  uint32_t commandSequence = 0;
  uint32_t startedMs = 0;
  uint32_t lastSendMs = 0;
  uint8_t attempts = 0;
  bool active = false;
};

struct ManualValues {
  float left = 0.0f;
  float right = 0.0f;
  float rear = 0.0f;
  float propulsion = 0.0f;
};

struct OperationConfig {
  bool waypointEnabled = false;
  bool attitudeEnabled = false;
  bool competitionCourse = false;
  bool placedWaypoints = false;
  uint8_t targetIndex = 0;
  uint8_t mode = boat::ControlManual;
};

struct TrackPoint {
  int32_t latitudeE7 = 0, longitudeE7 = 0;
  uint32_t capturedMs = 0;
};

struct DiagnosticEvent {
  uint32_t atMs = 0;
  char text[128]{};
};

struct RuntimeTuningValues {
  float kpPitch = 0.80f, kdPitch = 0.10f, kpRoll = 1.25f, kdRoll = 0.22f,
        pitchCommandCutoffHz = 3.0f, rollCommandCutoffHz = 3.0f, heightCommandCutoffHz = 3.0f,
        yawCommandCutoffHz = 3.0f, kpHeight = 0.75f, kpYaw = 0.90f, kdYaw = 0.12f,
        targetHeightM = 0.45f, maxDuty = 0.50f, vescRampRiseSeconds = 2.0f;
  float leftUpDeg = 105.0f, leftMiddleDeg = 90.0f, leftDownDeg = 75.0f;
  float rightUpDeg = 75.0f, rightMiddleDeg = 90.0f, rightDownDeg = 105.0f;
  float rearLeftDeg = 70.0f, rearMiddleDeg = 90.0f, rearRightDeg = 110.0f;
};

// BOATLOG1 is a compact, little-endian binary stream.  Quantized values keep
// each 1 Hz sample to 40 bytes (about 3.5 MB/day) without JSON/CSV overhead.
struct __attribute__((packed)) SdLogHeader {
  char magic[8] = {'B', 'O', 'A', 'T', 'L', 'O', 'G', '1'};
  uint16_t version = 1, recordBytes = 40;
  uint32_t bootId = 0, startMillis = 0;
};

struct __attribute__((packed)) SdLogRecord {
  uint32_t uptimeMs = 0;
  int32_t latitudeE7 = 0, longitudeE7 = 0, vescErpm = 0;
  int16_t speedCmPerS = 0, rollMilliRad = 0, pitchMilliRad = 0, tofMm = 0, leftMilli = 0,
          rightMilli = 0, rearMilli = 0, targetDutyTenThousandth = 0, appliedDutyTenThousandth = 0;
  uint16_t waypointDistanceCm = 0;
  uint8_t safety = 0, mode = 0, yawCriteria = 0, flags = 0;
};

static_assert(sizeof(SdLogHeader) == 20, "SD log header size");
static_assert(sizeof(SdLogRecord) == 40, "SD log record size");

HardwareSerial controlUart(1), gnssUart(2);
gnss::Receiver gnssRx;
WebServer web(kHttpPort);
boat::Decoder controlDecoder;
TaskHandle_t rxTaskHandle = nullptr;
portMUX_TYPE cacheMux = portMUX_INITIALIZER_UNLOCKED;
LinkCache linkCache{};

uint32_t bootId = 0;
uint32_t frameSequence = 0;
uint32_t requestIdNext = 1;
uint32_t commandSequenceNext = 1;
uint32_t waypointRevisionNext = 1;
uint32_t safetyCommandId = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastManualMs = 0;
uint32_t lastSafetyMs = 0;
uint32_t lastStatusLogMs = 0;
uint32_t crcErrors = 0;
uint32_t cobsErrors = 0;
uint32_t lengthErrors = 0;
uint32_t gnssNavSequence = 0;
uint32_t gnssFixSequence = 0;
uint32_t lastGnssNavMs = 0;
uint32_t gnssSentences = 0;
uint32_t gnssChecksumErrors = 0;
bool gnssNewFix = false;
bool sdReady = false;
bool sdLoggingEnabled = false;
File sdLogFile;
char sdLogPath[24] = "";
uint32_t lastSdLogMs = 0, sdLogRecords = 0, sdLogErrors = 0;
uint16_t sdLogSession = 0;
TrackPoint trackHistory[kTrackHistoryCapacity]{};
boat::WaypointGeo placedWaypoints[16]{};
uint16_t trackHistoryHead = 0, trackHistoryCount = 0;
uint8_t placedWaypointCount = 0;
uint32_t trackLastSampleMs = 0, trackRevision = 1;
DiagnosticEvent diagnosticEvents[16]{};
uint8_t diagnosticEventHead = 0, diagnosticEventCount = 0;

Stage stage = Stage::Idle;
PendingCommand pending{};
uint32_t stageStartedMs = 0;
ManualValues manualValues{};
OperationConfig operation{};
RuntimeTuningValues runtimeTuning{};
bool tuningOnlyPending = false;
char operationMessage[96] = "停止中です。";

uint64_t nowUs() {
  return static_cast<uint64_t>(esp_timer_get_time());
}

uint32_t ageMs(uint64_t timestampUs, uint64_t currentUs) {
  if (!timestampUs || currentUs < timestampUs)
    return UINT32_MAX;
  const uint64_t age = (currentUs - timestampUs) / 1000ULL;
  return age > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(age);
}

const char *stageName(Stage value) {
  switch (value) {
  case Stage::Idle:
    return "idle";
  case Stage::EnsureDisarmed:
    return "ensure_disarmed";
  case Stage::WaitTuningAck:
    return "wait_tuning_ack";
  case Stage::WaitWaypointAck:
    return "wait_waypoint_ack";
  case Stage::WaitModeAck:
    return "wait_mode_ack";
  case Stage::WaitManualAck:
    return "wait_manual_ack";
  case Stage::WaitArmed:
    return "wait_armed";
  case Stage::WaitRunning:
    return "wait_running";
  case Stage::Running:
    return "running";
  case Stage::Stopping:
    return "stopping";
  case Stage::Emergency:
    return "emergency";
  case Stage::ClearingEmergency:
    return "clearing_emergency";
  case Stage::Error:
    return "error";
  }
  return "unknown";
}

const char *modeName(uint8_t value) {
  switch (value) {
  case boat::ControlManual:
    return "MANUAL";
  case boat::ControlAttitudeAssist:
    return "ATTITUDE_ASSIST";
  case boat::ControlHeadingHold:
    return "HEADING_HOLD";
  case boat::ControlAutoWaypoint:
    return "AUTO_WAYPOINT_ATTITUDE";
  case boat::ControlWaypointOnly:
    return "AUTO_WAYPOINT_ONLY";
  default:
    return "UNKNOWN";
  }
}

bool usesWaypoint() {
  return operation.waypointEnabled;
}

bool usesManualRefresh() {
  return !operation.waypointEnabled;
}

uint8_t selectedMode(bool waypointEnabled, bool attitudeEnabled) {
  if (waypointEnabled) {
    return attitudeEnabled ? boat::ControlAutoWaypoint : boat::ControlWaypointOnly;
  }
  return attitudeEnabled ? boat::ControlAttitudeAssist : boat::ControlManual;
}

const char *safetyName(uint8_t value) {
  switch (value) {
  case 0:
    return "BOOT";
  case 1:
    return "DISARMED";
  case 2:
    return "ARMED";
  case 3:
    return "RUNNING";
  case 4:
    return "E-STOP";
  case 5:
    return "FAULT";
  default:
    return "UNKNOWN";
  }
}

const char *stopReasonName(uint8_t value) {
  switch (value) {
  case 0:
    return "NONE";
  case 1:
    return "STOP";
  case 2:
    return "E_STOP";
  case 3:
    return "HEARTBEAT";
  case 4:
    return "MANUAL_TIMEOUT";
  case 5:
    return "IMU_INVALID";
  case 6:
    return "IMU_STALE";
  case 7:
    return "TOF_INVALID";
  case 8:
    return "TOF_STALE";
  case 9:
    return "GNSS_INVALID";
  case 10:
    return "GNSS_STALE";
  case 11:
    return "VESC_FAULT";
  case 12:
    return "NONFINITE";
  case 13:
    return "FINAL_WAYPOINT";
  case 14:
    return "POWER_INVALID";
  case 15:
    return "POWER_STALE";
  case 16:
    return "LOW_VOLTAGE";
  case 17:
    return "OVER_CURRENT";
  case 18:
    return "MOTOR_STALL";
  case 19:
    return "ATTITUDE_DANGER";
  case 20:
    return "VESC_STALE";
  case 21:
    return "CAVITATION";
  default:
    return "UNKNOWN";
  }
}

void setMessage(const char *text) {
  snprintf(operationMessage, sizeof(operationMessage), "%s", text ? text : "");
}

void recordDiagnosticEvent(const char *text) {
  DiagnosticEvent &event = diagnosticEvents[diagnosticEventHead];
  event.atMs = millis();
  snprintf(event.text, sizeof(event.text), "%s", text ? text : "");
  diagnosticEventHead = static_cast<uint8_t>((diagnosticEventHead + 1) % 16);
  if (diagnosticEventCount < 16)
    ++diagnosticEventCount;
}

void setStage(Stage next, const char *message) {
  stage = next;
  stageStartedMs = millis();
  lastSafetyMs = 0;
  if (message)
    setMessage(message);
  if (message)
    recordDiagnosticEvent(message);
}

LinkCache cacheSnapshot() {
  LinkCache copy{};
  portENTER_CRITICAL(&cacheMux);
  copy = linkCache;
  portEXIT_CRITICAL(&cacheMux);
  return copy;
}

bool linkConnected(const LinkCache &cache) {
  return cache.lastFrameUs && ageMs(cache.lastFrameUs, nowUs()) <= kLinkFreshMs;
}

uint8_t currentSafety(const LinkCache &cache) {
  if (cache.hasActuators)
    return cache.actuators.safetyState;
  if (cache.hasOutput)
    return cache.output.safety;
  if (cache.hasHealth)
    return cache.health.safetyState;
  return 0;
}

uint8_t currentStopReason(const LinkCache &cache) {
  if (cache.hasOutput && cache.output.stopReason)
    return cache.output.stopReason;
  if (cache.hasHealth && cache.health.stopReason)
    return cache.health.stopReason;
  if (cache.hasSnapshot && cache.snapshot.safetyReason)
    return cache.snapshot.safetyReason;
  return cache.latchedStopReason;
}

bool sendFrame(boat::Type type, const void *payload, uint16_t length) {
  boat::Header header{
      boat::kVersion, static_cast<uint8_t>(type), length, ++frameSequence, bootId, nowUs(), 0};
  uint8_t encoded[boat::kMaxEncoded]{};
  const size_t bytes =
      boat::encode(header, static_cast<const uint8_t *>(payload), encoded, sizeof(encoded));
  return bytes && controlUart.write(encoded, bytes) == bytes;
}

bool gnssFixFresh() {
  const auto &fix = gnssRx.latest();
  const uint32_t required = gnss::FixValid | gnss::LatitudeValid | gnss::LongitudeValid;
  return (fix.flags & required) == required && fix.lastValidFixUs &&
         ageMs(fix.lastValidFixUs, nowUs()) <= kGnssFixFreshMs;
}

boat::GnssNavV2Payload makeGnssNav() {
  const auto &fix = gnssRx.latest();
  boat::GnssNavV2Payload nav{};
  nav.navSequence = ++gnssNavSequence;
  nav.fixSequence = gnssFixSequence;
  if (gnssFixFresh())
    nav.flags |= boat::NavFixValid;
  if (gnssNewFix)
    nav.flags |= boat::NavNewFix;
  if (fix.flags & gnss::LatitudeValid)
    nav.flags |= boat::NavLatValid;
  if (fix.flags & gnss::LongitudeValid)
    nav.flags |= boat::NavLonValid;
  if (fix.flags & gnss::AltitudeValid)
    nav.flags |= boat::NavAltitudeValid;
  if (fix.flags & gnss::SpeedValid)
    nav.flags |= boat::NavSpeedValid;
  if (fix.flags & gnss::CourseValid)
    nav.flags |= boat::NavCourseValid;
  if (fix.flags & gnss::HdopValid)
    nav.flags |= boat::NavHdopValid;
  nav.latitudeE7 = lround(fix.latitude * 1e7);
  nav.longitudeE7 = lround(fix.longitude * 1e7);
  nav.altitudeMm = lround(fix.altitudeM * 1000.0f);
  nav.speedMmPerSec = lround(fix.speedMps * 1000.0f);
  nav.courseE5Deg = lround(fix.courseDeg * 100000.0f);
  nav.hdopCenti = constrain(lround(fix.hdop * 100.0f), 0L, 65535L);
  nav.satellites = fix.satellites;
  nav.fixType = fix.fixType;
  nav.generatedUs = nowUs();
  nav.measurementUs = fix.lastValidFixUs ? fix.lastValidFixUs : nav.generatedUs;
  nav.sourceBootId = bootId;
  nav.canonicalCrc = boat::canonicalCrc(&nav, offsetof(boat::GnssNavV2Payload, canonicalCrc));
  return nav;
}

void serviceGnss() {
  for (uint16_t count = 0; count < kGnssReadBudgetBytes && gnssUart.available(); ++count) {
    const int value = gnssUart.read();
    if (value < 0)
      break;
    gnss::Sentence sentence{};
    if (gnssRx.feed(static_cast<char>(value), nowUs(), sentence)) {
      ++gnssSentences;
      if (!sentence.checksumValid)
        ++gnssChecksumErrors;
      if (sentence.parsed && (!strcmp(sentence.type, "RMC") || !strcmp(sentence.type, "GGA")) &&
          gnssFixFresh()) {
        ++gnssFixSequence;
        gnssNewFix = true;
      }
    }
  }
  gnssRx.expire(nowUs());
  if (millis() - lastGnssNavMs >= kGnssNavIntervalMs) {
    lastGnssNavMs = millis();
    const boat::GnssNavV2Payload nav = makeGnssNav();
    sendFrame(boat::Type::GnssNavV2, &nav, sizeof(nav));
    gnssNewFix = false;
  }
}

void initializePersistentCounters() {
  Preferences preferences;
  preferences.begin("boatcmd2", false);
  uint32_t request = preferences.getUInt("request", 0);
  uint32_t sequence = preferences.getUInt("sequence", 0);
  uint32_t waypointRevision = preferences.getUInt("wp_revision", 0);
  if (!request)
    request = esp_random() | 1U;
  if (!sequence)
    sequence = esp_random() | 1U;
  if (!waypointRevision)
    waypointRevision = esp_random() | 1U;
  constexpr uint32_t kReservation = 0x01000000UL;
  constexpr uint32_t kWaypointReservation = 0x00010000UL;
  preferences.putUInt("request", request + kReservation);
  preferences.putUInt("sequence", sequence + kReservation);
  preferences.putUInt("wp_revision", waypointRevision + kWaypointReservation);
  preferences.end();
  requestIdNext = request;
  commandSequenceNext = sequence;
  waypointRevisionNext = waypointRevision;
}

constexpr uint8_t kAllManualMask = boat::ManualAll;

boat::ManualCommandPayload makeManual(uint8_t mask, const ManualValues &values) {
  boat::ManualCommandPayload command{};
  command.protocolVersion = boat::kVersion;
  command.reserved[0] = mask;
  command.requestId = requestIdNext++;
  command.commandSequence = commandSequenceNext++;
  command.sourceUs = nowUs();
  if (mask & boat::ManualLeft)
    command.leftFrontWing = values.left;
  if (mask & boat::ManualRight)
    command.rightFrontWing = values.right;
  if (mask & boat::ManualRear)
    command.rearYaw = values.rear;
  if (mask & boat::ManualPropulsion)
    command.propulsion = values.propulsion;
  command.canonicalCrc =
      boat::canonicalCrc(&command, offsetof(boat::ManualCommandPayload, canonicalCrc));
  return command;
}

void beginPending(boat::Type type,
                  const void *payload,
                  uint16_t length,
                  uint32_t requestId,
                  uint32_t commandSequence) {
  pending = {};
  pending.type = type;
  pending.length = length;
  pending.requestId = requestId;
  pending.commandSequence = commandSequence;
  pending.startedMs = millis();
  pending.active = true;
  memcpy(pending.payload, payload, length);
  if (sendFrame(type, payload, length)) {
    pending.lastSendMs = millis();
    pending.attempts = 1;
  }
}

void beginModeCommand() {
  boat::ControlModeCommandPayload command{};
  command.protocolVersion = boat::kVersion;
  command.mode = operation.mode;
  command.requestId = requestIdNext++;
  command.commandSequence = commandSequenceNext++;
  command.sourceUs = nowUs();
  command.canonicalCrc =
      boat::canonicalCrc(&command, offsetof(boat::ControlModeCommandPayload, canonicalCrc));
  beginPending(boat::Type::ControlModeCommand,
               &command,
               sizeof(command),
               command.requestId,
               command.commandSequence);
}

void beginWaypointCommand() {
  const fixed_waypoints::Point *point = fixed_waypoints::byIndex(operation.targetIndex);
  if (!operation.competitionCourse && !operation.placedWaypoints && !point) {
    pending.active = false;
    setMessage("固定ウェイポイントの選択が不正です。");
    return;
  }
  if (operation.placedWaypoints && !placedWaypointCount) {
    pending.active = false;
    setMessage("配置したウェイポイントがありません。");
    return;
  }
  boat::WaypointSetPayload command{};
  command.requestId = requestIdNext++;
  command.revision = waypointRevisionNext++;
  command.action = (operation.competitionCourse || operation.placedWaypoints) ? 2 : 1;
  command.count = operation.competitionCourse
                      ? static_cast<uint8_t>(fixed_waypoints::kCompetitionRouteCount)
                      : (operation.placedWaypoints ? placedWaypointCount : 1);
  command.reachRadiusM = kWaypointReachRadiusM;
  for (uint8_t i = 0; i < command.count; ++i) {
    if (operation.placedWaypoints)
      command.points[i] = placedWaypoints[i];
    else {
      const fixed_waypoints::Point *source =
          operation.competitionCourse
              ? fixed_waypoints::byIndex(fixed_waypoints::kCompetitionRouteIndices[i])
              : point;
      command.points[i].latitudeDeg = source->latitudeDeg;
      command.points[i].longitudeDeg = source->longitudeDeg;
    }
  }
  command.canonicalCrc =
      boat::canonicalCrc(&command, offsetof(boat::WaypointSetPayload, canonicalCrc));
  beginPending(
      boat::Type::WaypointSet, &command, sizeof(command), command.requestId, command.revision);
}

void beginManualCommand() {
  const boat::ManualCommandPayload command = makeManual(kAllManualMask, manualValues);
  beginPending(boat::Type::ManualCommand,
               &command,
               sizeof(command),
               command.requestId,
               command.commandSequence);
}

int16_t logI16(float value, float scale) {
  if (!isfinite(value))
    return 0;
  const float scaled = value * scale;
  return scaled < -32768.0f ? -32768
                            : (scaled > 32767.0f ? 32767 : static_cast<int16_t>(lroundf(scaled)));
}

int32_t logI32(double value, double scale) {
  if (!isfinite(value))
    return 0;
  const double scaled = value * scale;
  return scaled < static_cast<double>(INT32_MIN)
             ? INT32_MIN
             : (scaled > static_cast<double>(INT32_MAX) ? INT32_MAX
                                                        : static_cast<int32_t>(llround(scaled)));
}

uint16_t logU16(float value, float scale) {
  if (!isfinite(value) || value <= 0.0f)
    return 0;
  const float scaled = value * scale;
  return scaled >= 65535.0f ? 65535 : static_cast<uint16_t>(lroundf(scaled));
}

bool beginSdLog() {
  if (!sdReady)
    return false;
  if (sdLoggingEnabled)
    return true;
  ++sdLogSession;
  sdLogRecords = 0;
  lastSdLogMs = 0;
  snprintf(sdLogPath,
           sizeof(sdLogPath),
           "/BOAT_%08lX_%02u.BIN",
           static_cast<unsigned long>(bootId),
           static_cast<unsigned>(sdLogSession));
  sdLogFile = SD.open(sdLogPath, FILE_WRITE);
  if (!sdLogFile) {
    ++sdLogErrors;
    return false;
  }
  SdLogHeader header{};
  header.bootId = bootId;
  header.startMillis = millis();
  if (sdLogFile.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header)) !=
      sizeof(header)) {
    ++sdLogErrors;
    sdLogFile.close();
    return false;
  }
  sdLogFile.flush();
  sdLoggingEnabled = true;
  return true;
}

void stopSdLog() {
  if (sdLogFile) {
    sdLogFile.flush();
    sdLogFile.close();
  }
  sdLoggingEnabled = false;
}

void serviceSdLog() {
  if (!sdLoggingEnabled || !sdLogFile || millis() - lastSdLogMs < kSdLogIntervalMs)
    return;
  lastSdLogMs = millis();
  const LinkCache cache = cacheSnapshot();
  const auto &fix = gnssRx.latest();
  SdLogRecord record{};
  record.uptimeMs = lastSdLogMs;
  record.latitudeE7 = logI32(fix.latitude, 1e7);
  record.longitudeE7 = logI32(fix.longitude, 1e7);
  record.speedCmPerS = logI16(fix.speedMps, 100.0f);
  if (cache.hasSnapshot) {
    const auto &s = cache.snapshot;
    record.rollMilliRad = logI16(s.rollRad, 1000.0f);
    record.pitchMilliRad = logI16(s.pitchRad, 1000.0f);
    record.tofMm = logI16(s.tofFilteredM, 1000.0f);
    record.leftMilli = logI16(s.leftFrontWing, 1000.0f);
    record.rightMilli = logI16(s.rightFrontWing, 1000.0f);
    record.rearMilli = logI16(s.rearYaw, 1000.0f);
    record.vescErpm = logI32(s.vescErpm, 1.0);
    record.safety = s.state;
    record.mode = s.mode;
    record.yawCriteria = s.yawCriteriaMask;
    record.waypointDistanceCm = logU16(s.waypointDistanceM, 100.0f);
    record.flags |= 16u | (s.gnssValid ? 1u : 0u) | (s.imuValid ? 2u : 0u) |
                    (s.tofValid ? 4u : 0u) | (s.yawControlActive ? 8u : 0u);
  }
  if (cache.hasActuators) {
    record.targetDutyTenThousandth = logI16(cache.actuators.targetDuty, 10000.0f);
    record.appliedDutyTenThousandth = logI16(cache.actuators.appliedDuty, 10000.0f);
  }
  if (linkConnected(cache))
    record.flags |= 32u;
  if (sdLogFile.write(reinterpret_cast<const uint8_t *>(&record), sizeof(record)) !=
      sizeof(record)) {
    ++sdLogErrors;
    stopSdLog();
    return;
  }
  ++sdLogRecords;
  if (sdLogRecords % kSdLogFlushEveryRecords == 0)
    sdLogFile.flush();
}

void serviceTrack() {
  if (!gnssFixFresh() ||
      (trackLastSampleMs && millis() - trackLastSampleMs < kTrackCoarseIntervalMs))
    return;
  const auto &fix = gnssRx.latest();
  if (!isfinite(fix.latitude) || !isfinite(fix.longitude) || fix.latitude < -90.0 ||
      fix.latitude > 90.0 || fix.longitude < -180.0 || fix.longitude > 180.0)
    return;
  uint16_t slot = 0;
  if (trackHistoryCount < kTrackHistoryCapacity) {
    slot = static_cast<uint16_t>((trackHistoryHead + trackHistoryCount) % kTrackHistoryCapacity);
    ++trackHistoryCount;
  } else {
    slot = trackHistoryHead;
    trackHistoryHead = (trackHistoryHead + 1) % kTrackHistoryCapacity;
  }
  trackHistory[slot].latitudeE7 = logI32(fix.latitude, 1e7);
  trackHistory[slot].longitudeE7 = logI32(fix.longitude, 1e7);
  trackHistory[slot].capturedMs = millis();
  trackLastSampleMs = millis();
  ++trackRevision;
}

void serviceDiagnostics() {
  static bool initialized = false, lastLink = false, lastGnss = false;
  static uint8_t lastSafety = 0xFF, lastReason = 0xFF;
  static uint32_t lastCrc = 0, lastCobs = 0, lastLength = 0;
  const LinkCache cache = cacheSnapshot();
  const bool link = linkConnected(cache);
  const bool gnss = gnssFixFresh();
  const uint8_t safety = currentSafety(cache);
  const uint8_t reason = currentStopReason(cache);
  if (!initialized) {
    initialized = true;
    recordDiagnosticEvent("通信側XIAOを起動しました。");
    recordDiagnosticEvent(sdReady ? "SDカードを認識しました。" : "SDカードを認識していません。");
    lastLink = link;
    lastGnss = gnss;
    lastSafety = safety;
    lastReason = reason;
    lastCrc = crcErrors;
    lastCobs = cobsErrors;
    lastLength = lengthErrors;
    if (!cache.lastFrameUs)
      recordDiagnosticEvent("制御側UARTは未受信です。電源・D7→D1・D0→D6・GNDを確認してください。");
    return;
  }
  if (link != lastLink) {
    recordDiagnosticEvent(link ? "制御側XIAOからのUART受信を開始しました。"
                               : "制御側UARTフレームが1秒以上途絶しました。");
    lastLink = link;
  }
  if (gnss != lastGnss) {
    recordDiagnosticEvent(gnss ? "GNSS測位が有効になりました。"
                               : "GNSS測位が無効または古くなりました。");
    lastGnss = gnss;
  }
  if (link && safety != lastSafety) {
    char text[88];
    snprintf(text, sizeof(text), "制御側の安全状態: %s", safetyName(safety));
    recordDiagnosticEvent(text);
    lastSafety = safety;
  }
  if (reason != lastReason) {
    if (reason) {
      char text[88];
      snprintf(text, sizeof(text), "制御側の停止・制限理由: %s", stopReasonName(reason));
      recordDiagnosticEvent(text);
    }
    lastReason = reason;
  }
  if (crcErrors != lastCrc || cobsErrors != lastCobs || lengthErrors != lastLength) {
    char text[88];
    snprintf(text,
             sizeof(text),
             "UART受信エラー CRC:%lu COBS:%lu LEN:%lu",
             static_cast<unsigned long>(crcErrors),
             static_cast<unsigned long>(cobsErrors),
             static_cast<unsigned long>(lengthErrors));
    recordDiagnosticEvent(text);
    lastCrc = crcErrors;
    lastCobs = cobsErrors;
    lastLength = lengthErrors;
  }
}

boat::RuntimeTuningCommandPayload makeTuningCommand(const RuntimeTuningValues &values) {
  boat::RuntimeTuningCommandPayload command{};
  command.protocolVersion = boat::kVersion;
  command.requestId = requestIdNext++;
  command.commandSequence = commandSequenceNext++;
  command.sourceUs = nowUs();
  command.kpPitch = values.kpPitch;
  command.kdPitch = values.kdPitch;
  command.kpRoll = values.kpRoll;
  command.kdRoll = values.kdRoll;
  command.pitchCommandCutoffHz = values.pitchCommandCutoffHz;
  command.rollCommandCutoffHz = values.rollCommandCutoffHz;
  command.heightCommandCutoffHz = values.heightCommandCutoffHz;
  command.yawCommandCutoffHz = values.yawCommandCutoffHz;
  command.kpHeight = values.kpHeight;
  command.kpYaw = values.kpYaw;
  command.kdYaw = values.kdYaw;
  command.targetHeightM = values.targetHeightM;
  command.maxDuty = values.maxDuty;
  command.vescRampRiseSeconds = values.vescRampRiseSeconds;
  command.leftUpDeg = values.leftUpDeg;
  command.leftMiddleDeg = values.leftMiddleDeg;
  command.leftDownDeg = values.leftDownDeg;
  command.rightUpDeg = values.rightUpDeg;
  command.rightMiddleDeg = values.rightMiddleDeg;
  command.rightDownDeg = values.rightDownDeg;
  command.rearLeftDeg = values.rearLeftDeg;
  command.rearMiddleDeg = values.rearMiddleDeg;
  command.rearRightDeg = values.rearRightDeg;
  command.canonicalCrc =
      boat::canonicalCrc(&command, offsetof(boat::RuntimeTuningCommandPayload, canonicalCrc));
  return command;
}

void beginTuningCommand() {
  const boat::RuntimeTuningCommandPayload command = makeTuningCommand(runtimeTuning);
  beginPending(boat::Type::RuntimeTuning,
               &command,
               sizeof(command),
               command.requestId,
               command.commandSequence);
}

void sendManualRefresh() {
  const boat::ManualCommandPayload command = makeManual(kAllManualMask, manualValues);
  if (sendFrame(boat::Type::ManualCommand, &command, sizeof(command))) {
    lastManualMs = millis();
  }
}

void sendManualOff() {
  const boat::ManualCommandPayload command = makeManual(0, ManualValues{});
  sendFrame(boat::Type::ManualCommand, &command, sizeof(command));
}

void sendSafety(boat::Type type) {
  boat::CommandPayload command{++safetyCommandId, static_cast<uint8_t>(type), {0, 0, 0}};
  if (sendFrame(type, &command, sizeof(command)))
    lastSafetyMs = millis();
}

// Returns 1 when accepted, -1 when rejected/timed out, and 0 while waiting.
int servicePending(const LinkCache &cache) {
  if (!pending.active)
    return -1;
  if (pending.type == boat::Type::WaypointSet && cache.hasWaypointAck &&
      cache.waypointAck.requestId == pending.requestId &&
      cache.waypointAck.revision == pending.commandSequence) {
    const bool crcOk =
        cache.waypointAck.canonicalCrc ==
        boat::canonicalCrc(&cache.waypointAck, offsetof(boat::WaypointAckPayload, canonicalCrc));
    pending.active = false;
    if (crcOk && (cache.waypointAck.status == 0 || cache.waypointAck.status == 2)) {
      return 1;
    }
    char message[96];
    snprintf(message,
             sizeof(message),
             "XIAOがウェイポイントを拒否しました（理由%u）。",
             static_cast<unsigned>(cache.waypointAck.reason));
    setMessage(message);
    return -1;
  }
  if (cache.hasCommandAck && cache.commandAck.requestId == pending.requestId &&
      cache.commandAck.commandSequence == pending.commandSequence &&
      cache.commandAck.commandType == static_cast<uint8_t>(pending.type)) {
    pending.active = false;
    if (cache.commandAck.disposition == 0 ||
        (cache.commandAck.disposition == 2 && cache.commandAck.reason == 0)) {
      return 1;
    }
    char message[96];
    snprintf(message,
             sizeof(message),
             "XIAOが指令を拒否しました（理由%u）。",
             static_cast<unsigned>(cache.commandAck.reason));
    setMessage(message);
    return -1;
  }

  const uint32_t current = millis();
  if (current - pending.startedMs > kCommandTimeoutMs) {
    pending.active = false;
    setMessage("XIAOから指令ACKが返りませんでした。");
    return -1;
  }
  if ((!pending.lastSendMs || current - pending.lastSendMs >= kCommandRetryMs) &&
      pending.attempts < 8) {
    if (sendFrame(pending.type, pending.payload, pending.length)) {
      pending.lastSendMs = current;
      ++pending.attempts;
    }
  }
  return 0;
}

void failOperation(const char *message) {
  char savedMessage[sizeof(operationMessage)];
  snprintf(savedMessage, sizeof(savedMessage), "%s", message ? message : operationMessage);
  pending.active = false;
  sendSafety(boat::Type::Stop);
  sendManualOff();
  setStage(Stage::Error, savedMessage);
}

void failSafetyOperation(const LinkCache &cache, const char *phase) {
  char message[sizeof(operationMessage)];
  const uint8_t reason = currentStopReason(cache);
  if (reason) {
    snprintf(message, sizeof(message), "%s中に安全停止: %s", phase, stopReasonName(reason));
  } else if (cache.hasActuators && cache.actuators.pwmErrors) {
    snprintf(message, sizeof(message), "%s中に安全停止: PCA9685_WRITE", phase);
  } else {
    snprintf(message, sizeof(message), "%s中に安全停止: 理由未受信", phase);
  }
  failOperation(message);
}

void keepManualFresh() {
  if (!lastManualMs || millis() - lastManualMs >= kManualRefreshMs) {
    sendManualRefresh();
  }
}

void serviceOperation() {
  const LinkCache cache = cacheSnapshot();
  const bool connected = linkConnected(cache);
  const uint8_t safety = currentSafety(cache);
  const uint32_t current = millis();

  if (stage != Stage::Idle && stage != Stage::Error && !connected) {
    failOperation("XIAOとの通信が途切れたため停止指令を送りました。");
    return;
  }

  switch (stage) {
  case Stage::Idle:
    return;

  case Stage::Error:
    if (connected && safety != 1 && safety != 4 &&
        (!lastSafetyMs || current - lastSafetyMs >= kSafetyRetryMs)) {
      sendSafety(boat::Type::Stop);
      sendManualOff();
    }
    return;

  case Stage::EnsureDisarmed:
    if (safety == 4) {
      failOperation("緊急停止中です。解除してから開始してください。");
    } else if (safety == 1) {
      beginTuningCommand();
      setStage(Stage::WaitTuningAck, "制御ゲイン・高さ・サーボ角をXIAOへ設定しています。");
    } else if (!lastSafetyMs || current - lastSafetyMs >= kSafetyRetryMs) {
      sendSafety(boat::Type::Stop);
      if (current - stageStartedMs > kSafetyTimeoutMs) {
        failOperation("XIAOをDISARMEDにできませんでした。");
      }
    }
    return;

  case Stage::WaitTuningAck: {
    const int result = servicePending(cache);
    if (result > 0) {
      if (tuningOnlyPending) {
        tuningOnlyPending = false;
        setStage(Stage::Idle, "調整値をXIAOへ適用しました。");
      } else if (usesWaypoint()) {
        beginWaypointCommand();
        setStage(Stage::WaitWaypointAck, "固定ウェイポイントをXIAOへ設定しています。");
      } else {
        beginModeCommand();
        setStage(Stage::WaitModeAck, "制御モードを設定しています。");
      }
    } else if (result < 0) {
      tuningOnlyPending = false;
      failOperation(operationMessage);
    }
    return;
  }

  case Stage::WaitWaypointAck: {
    const int result = servicePending(cache);
    if (result > 0) {
      beginModeCommand();
      setStage(Stage::WaitModeAck, "自動航法モードを設定しています。");
    } else if (result < 0) {
      failOperation(operationMessage);
    }
    return;
  }

  case Stage::WaitModeAck: {
    const int result = servicePending(cache);
    if (result > 0) {
      if (usesManualRefresh()) {
        beginManualCommand();
        setStage(Stage::WaitManualAck, "手動入力値を設定しています。");
      } else {
        sendManualOff();
        sendSafety(boat::Type::Arm);
        setStage(Stage::WaitArmed, "ARMの成立を待っています。");
      }
    } else if (result < 0) {
      failOperation(operationMessage);
    }
    return;
  }

  case Stage::WaitManualAck: {
    const int result = servicePending(cache);
    if (result > 0) {
      lastManualMs = current;
      sendSafety(boat::Type::Arm);
      setStage(Stage::WaitArmed, "ARMの成立を待っています。");
    } else if (result < 0) {
      failOperation(operationMessage);
    }
    return;
  }

  case Stage::WaitArmed:
    if (usesManualRefresh())
      keepManualFresh();
    if (safety == 2) {
      sendSafety(boat::Type::StartTest);
      setStage(Stage::WaitRunning, "STARTの成立を待っています。");
    } else if (safety == 4) {
      failOperation("ARM中に緊急停止になりました。");
    } else if (current - stageStartedMs > kSafetyTimeoutMs) {
      failOperation("ARMできませんでした。制御側の事前条件を確認してください。");
    } else if (!lastSafetyMs || current - lastSafetyMs >= kSafetyRetryMs) {
      sendSafety(boat::Type::Arm);
    }
    return;

  case Stage::WaitRunning:
    if (usesManualRefresh())
      keepManualFresh();
    if (safety == 3) {
      setStage(Stage::Running,
               usesWaypoint()
                   ? "固定ウェイポイントへの自動航行を実行しています。"
                   : (operation.attitudeEnabled ? "姿勢補助付き手動運転を実行しています。"
                                                : "全サーボと推進を手動出力しています。"));
    } else if (safety == 4 || safety == 5) {
      failSafetyOperation(cache, "START");
    } else if (current - stageStartedMs > kSafetyTimeoutMs) {
      failOperation("STARTできませんでした。XIAOの状態を確認してください。");
    } else if (!lastSafetyMs || current - lastSafetyMs >= kSafetyRetryMs) {
      sendSafety(boat::Type::StartTest);
    }
    return;

  case Stage::Running:
    if (usesManualRefresh())
      keepManualFresh();
    if (usesWaypoint() && safety == 1 && cache.sawFinalWaypoint) {
      sendManualOff();
      setStage(Stage::Idle, "選択した固定ウェイポイントへ到着し、全出力を停止しました。");
    } else if (safety != 3) {
      failSafetyOperation(cache, "RUNNING");
    }
    return;

  case Stage::Stopping:
    if (safety == 1) {
      sendManualOff();
      setStage(Stage::Idle, "停止しました。すべての出力はOFFです。");
    } else if (safety == 4) {
      setStage(Stage::Emergency, "緊急停止中です。すべての出力はOFFです。");
    } else if (!lastSafetyMs || current - lastSafetyMs >= kSafetyRetryMs) {
      sendSafety(boat::Type::Stop);
    }
    return;

  case Stage::Emergency:
    if (safety != 4 && current - stageStartedMs > kSafetyTimeoutMs) {
      setMessage("緊急停止状態を確認できません。物理的に電源を切ってください。");
    } else if (safety != 4 && (!lastSafetyMs || current - lastSafetyMs >= kSafetyRetryMs)) {
      sendSafety(boat::Type::Estop);
    }
    return;

  case Stage::ClearingEmergency:
    if (safety == 1) {
      setStage(Stage::Idle, "緊急停止を解除しました。出力はOFFです。");
    } else if (current - stageStartedMs > kSafetyTimeoutMs) {
      failOperation("緊急停止を解除できませんでした。");
    } else if (!lastSafetyMs || current - lastSafetyMs >= kSafetyRetryMs) {
      sendSafety(boat::Type::ClearEstop);
    }
    return;
  }
}

void processFrame(const boat::Frame &frame) {
  const boat::Type type = static_cast<boat::Type>(frame.header.type);
  const uint64_t receivedUs = nowUs();
  portENTER_CRITICAL(&cacheMux);
  linkCache.lastFrameUs = receivedUs;
  ++linkCache.frames;
  if (type == boat::Type::ControlSnapshot && frame.header.length == sizeof(linkCache.snapshot)) {
    memcpy(&linkCache.snapshot, frame.payload, sizeof(linkCache.snapshot));
    linkCache.lastSnapshotUs = receivedUs;
    linkCache.hasSnapshot = true;
    if (linkCache.snapshot.tofValid &&
        (linkCache.latchedStopReason == 7 || linkCache.latchedStopReason == 8))
      linkCache.latchedStopReason = 0;
    if (linkCache.snapshot.safetyReason)
      linkCache.latchedStopReason = linkCache.snapshot.safetyReason;
  } else if (type == boat::Type::ControlOutput && frame.header.length == sizeof(linkCache.output)) {
    memcpy(&linkCache.output, frame.payload, sizeof(linkCache.output));
    linkCache.lastOutputUs = receivedUs;
    linkCache.hasOutput = true;
    if (linkCache.output.stopReason)
      linkCache.latchedStopReason = linkCache.output.stopReason;
    if (linkCache.output.stopReason == 13)
      linkCache.sawFinalWaypoint = true;
  } else if (type == boat::Type::ActuatorState &&
             frame.header.length == sizeof(linkCache.actuators)) {
    memcpy(&linkCache.actuators, frame.payload, sizeof(linkCache.actuators));
    linkCache.lastActuatorUs = receivedUs;
    linkCache.hasActuators = true;
  } else if (type == boat::Type::SystemHealth && frame.header.length == sizeof(linkCache.health)) {
    memcpy(&linkCache.health, frame.payload, sizeof(linkCache.health));
    linkCache.hasHealth = true;
    if (linkCache.health.stopReason)
      linkCache.latchedStopReason = linkCache.health.stopReason;
  } else if (type == boat::Type::InaStatus && frame.header.length == sizeof(linkCache.ina)) {
    memcpy(&linkCache.ina, frame.payload, sizeof(linkCache.ina));
    linkCache.lastInaStatusUs = receivedUs;
    linkCache.hasIna = true;
  } else if (type == boat::Type::VescTelemetry && frame.header.length == sizeof(linkCache.vesc)) {
    memcpy(&linkCache.vesc, frame.payload, sizeof(linkCache.vesc));
    linkCache.lastVescTelemetryUs = receivedUs;
    linkCache.hasVesc = true;
  } else if (type == boat::Type::ControlCommandAck &&
             frame.header.length == sizeof(linkCache.commandAck)) {
    memcpy(&linkCache.commandAck, frame.payload, sizeof(linkCache.commandAck));
    linkCache.lastAckUs = receivedUs;
    linkCache.hasCommandAck = true;
  } else if (type == boat::Type::WaypointAck &&
             frame.header.length == sizeof(linkCache.waypointAck)) {
    memcpy(&linkCache.waypointAck, frame.payload, sizeof(linkCache.waypointAck));
    linkCache.lastWaypointAckUs = receivedUs;
    linkCache.hasWaypointAck = true;
  }
  portEXIT_CRITICAL(&cacheMux);
}

void controlRxTask(void *) {
  uint8_t bytes[kRxChunkBytes];
  for (;;) {
    const int available = controlUart.available();
    if (available <= 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    const size_t wanted = min<size_t>(static_cast<size_t>(available), sizeof(bytes));
    const size_t count = controlUart.read(bytes, wanted);
    for (size_t index = 0; index < count; ++index) {
      boat::Frame frame{};
      if (controlDecoder.feed(bytes[index], frame))
        processFrame(frame);
    }
    crcErrors = controlDecoder.crcErrors;
    cobsErrors = controlDecoder.cobsErrors;
    lengthErrors = controlDecoder.lengthErrors;
  }
}

void sendHeartbeat() {
  const uint32_t current = millis();
  if (current - lastHeartbeatMs < kHeartbeatPeriodMs)
    return;
  const LinkCache cache = cacheSnapshot();
  // A heartbeat is proof of a healthy bidirectional link. If the communication
  // XIAO can no
  // longer receive XIAO telemetry, stop heartbeats so XIAO's own link timeout
  // also forces every physical output off.
  if (!linkConnected(cache))
    return;
  lastHeartbeatMs = current;
  boat::HeartbeatPayload heartbeat{current, frameSequence, currentSafety(cache), 0, 0};
  sendFrame(boat::Type::Heartbeat, &heartbeat, sizeof(heartbeat));
}

bool parseFloatArgument(const char *name, float &value) {
  const String source = web.arg(name);
  if (!source.length())
    return false;
  char *end = nullptr;
  value = strtof(source.c_str(), &end);
  return end && *end == 0 && isfinite(value);
}

bool parseManualValues(ManualValues &values) {
  if (!parseFloatArgument("left", values.left) || !parseFloatArgument("right", values.right) ||
      !parseFloatArgument("rear", values.rear) ||
      !parseFloatArgument("propulsion", values.propulsion)) {
    return false;
  }
  return values.left >= -1.0f && values.left <= 1.0f && values.right >= -1.0f &&
         values.right <= 1.0f && values.rear >= -1.0f && values.rear <= 1.0f &&
         values.propulsion >= 0.0f && values.propulsion <= 1.0f;
}

bool parseBooleanArgument(const char *name, bool &value) {
  const String source = web.arg(name);
  if (source == "1" || source == "true") {
    value = true;
    return true;
  }
  if (source == "0" || source == "false") {
    value = false;
    return true;
  }
  return false;
}

bool parseOperationConfig(OperationConfig &config) {
  if (!parseBooleanArgument("waypoint", config.waypointEnabled) ||
      !parseBooleanArgument("attitude", config.attitudeEnabled))
    return false;
  if (web.hasArg("competition_course") &&
      !parseBooleanArgument("competition_course", config.competitionCourse))
    return false;
  const String target = web.arg("target");
  if (target == "placed") {
    if (!placedWaypointCount)
      return false;
    config.placedWaypoints = true;
    config.targetIndex = 0;
    config.mode = selectedMode(config.waypointEnabled, config.attitudeEnabled);
    return true;
  }
  if (config.competitionCourse) {
    config.targetIndex = 0;
    config.mode = selectedMode(config.waypointEnabled, config.attitudeEnabled);
    return target == "course";
  }
  char *end = nullptr;
  const long index = strtol(target.c_str(), &end, 10);
  if (!target.length() || !end || *end || index < 0 ||
      index >= static_cast<long>(fixed_waypoints::kCount))
    return false;
  config.targetIndex = static_cast<uint8_t>(index);
  config.mode = selectedMode(config.waypointEnabled, config.attitudeEnabled);
  return true;
}

bool validRange(float value, float low, float high) {
  return isfinite(value) && value >= low && value <= high;
}

bool validServoPositions(float negative, float center, float positive) {
  return validRange(negative, 0.0f, 180.0f) && validRange(center, 0.0f, 180.0f) &&
         validRange(positive, 0.0f, 180.0f);
}

bool parseRuntimeTuning(RuntimeTuningValues &v) {
  if (!parseFloatArgument("kp_pitch", v.kpPitch) || !parseFloatArgument("kd_pitch", v.kdPitch) ||
      !parseFloatArgument("kp_roll", v.kpRoll) || !parseFloatArgument("kd_roll", v.kdRoll) ||
      !parseFloatArgument("pitch_cutoff_hz", v.pitchCommandCutoffHz) ||
      !parseFloatArgument("roll_cutoff_hz", v.rollCommandCutoffHz) ||
      !parseFloatArgument("height_cutoff_hz", v.heightCommandCutoffHz) ||
      !parseFloatArgument("yaw_cutoff_hz", v.yawCommandCutoffHz) ||
      !parseFloatArgument("kp_height", v.kpHeight) || !parseFloatArgument("kp_yaw", v.kpYaw) ||
      !parseFloatArgument("kd_yaw", v.kdYaw) ||
      !parseFloatArgument("target_height", v.targetHeightM) ||
      !parseFloatArgument("max_duty", v.maxDuty) ||
      !parseFloatArgument("vesc_ramp_rise_s", v.vescRampRiseSeconds) ||
      !parseFloatArgument("left_up", v.leftUpDeg) ||
      !parseFloatArgument("left_middle", v.leftMiddleDeg) ||
      !parseFloatArgument("left_down", v.leftDownDeg) ||
      !parseFloatArgument("right_up", v.rightUpDeg) ||
      !parseFloatArgument("right_middle", v.rightMiddleDeg) ||
      !parseFloatArgument("right_down", v.rightDownDeg) ||
      !parseFloatArgument("rear_left", v.rearLeftDeg) ||
      !parseFloatArgument("rear_middle", v.rearMiddleDeg) ||
      !parseFloatArgument("rear_right", v.rearRightDeg))
    return false;
  return validRange(v.kpPitch, 0.0f, 10.0f) && validRange(v.kdPitch, 0.0f, 5.0f) &&
         validRange(v.kpRoll, 0.0f, 10.0f) && validRange(v.kdRoll, 0.0f, 5.0f) &&
         validRange(v.pitchCommandCutoffHz, 0.0f, 10.0f) &&
         validRange(v.rollCommandCutoffHz, 0.0f, 10.0f) &&
         validRange(v.heightCommandCutoffHz, 0.0f, 10.0f) &&
         validRange(v.yawCommandCutoffHz, 0.0f, 10.0f) && validRange(v.kpHeight, 0.0f, 10.0f) &&
         validRange(v.kpYaw, 0.0f, 10.0f) && validRange(v.kdYaw, 0.0f, 5.0f) &&
         validRange(v.targetHeightM, 0.10f, 2.00f) && validRange(v.maxDuty, 0.0f, 1.00f) &&
         validRange(v.vescRampRiseSeconds, 0.10f, 20.0f) &&
         validServoPositions(v.leftUpDeg, v.leftMiddleDeg, v.leftDownDeg) &&
         validServoPositions(v.rightUpDeg, v.rightMiddleDeg, v.rightDownDeg) &&
         validServoPositions(v.rearLeftDeg, v.rearMiddleDeg, v.rearRightDeg);
}

void sendJsonResult(int code, bool accepted, const char *message) {
  char body[180];
  snprintf(body,
           sizeof(body),
           "{\"accepted\":%s,\"message\":\"%s\"}",
           accepted ? "true" : "false",
           message);
  web.send(code, "application/json; charset=utf-8", body);
}

void apiTuningGet() {
  char body[1000];
  snprintf(
      body,
      sizeof(body),
      "{\"kp_pitch\":%.4f,\"kd_pitch\":%.4f,\"kp_roll\":%.4f,\"kd_roll\":%.4f,\"pitch_cutoff_hz\":%.4f,\"roll_cutoff_hz\":%.4f,\"height_cutoff_hz\":%.4f,\"yaw_cutoff_hz\":%.4f,\"kp_height\":%.4f,\"kp_yaw\":%.4f,\"kd_yaw\":%.4f,\"target_height\":%.4f,\"max_duty\":%.4f,\"vesc_ramp_rise_s\":%.4f,\"left\":{\"up\":%.2f,\"middle\":%.2f,\"down\":%.2f},\"right\":{\"up\":%.2f,\"middle\":%.2f,\"down\":%.2f},\"rear\":{\"left\":%.2f,\"middle\":%.2f,\"right\":%.2f},\"persistent\":false}",
      runtimeTuning.kpPitch,
      runtimeTuning.kdPitch,
      runtimeTuning.kpRoll,
      runtimeTuning.kdRoll,
      runtimeTuning.pitchCommandCutoffHz,
      runtimeTuning.rollCommandCutoffHz,
      runtimeTuning.heightCommandCutoffHz,
      runtimeTuning.yawCommandCutoffHz,
      runtimeTuning.kpHeight,
      runtimeTuning.kpYaw,
      runtimeTuning.kdYaw,
      runtimeTuning.targetHeightM,
      runtimeTuning.maxDuty,
      runtimeTuning.vescRampRiseSeconds,
      runtimeTuning.leftUpDeg,
      runtimeTuning.leftMiddleDeg,
      runtimeTuning.leftDownDeg,
      runtimeTuning.rightUpDeg,
      runtimeTuning.rightMiddleDeg,
      runtimeTuning.rightDownDeg,
      runtimeTuning.rearLeftDeg,
      runtimeTuning.rearMiddleDeg,
      runtimeTuning.rearRightDeg);
  web.send(200, "application/json; charset=utf-8", body);
}

void apiTuningPost() {
  RuntimeTuningValues requested{};
  if (!parseRuntimeTuning(requested)) {
    sendJsonResult(400, false, "調整値が範囲外、または最小＜中立＜最大になっていません。");
    return;
  }
  const LinkCache cache = cacheSnapshot();
  if (!linkConnected(cache)) {
    sendJsonResult(503, false, "XIAOと通信できていません。");
    return;
  }
  if (stage != Stage::Idle && stage != Stage::Error) {
    sendJsonResult(409, false, "停止中だけ調整値を変更できます。");
    return;
  }
  if (currentSafety(cache) != 1) {
    sendJsonResult(409, false, "制御側XIAOをDISARMEDにしてから変更してください。");
    return;
  }
  runtimeTuning = requested;
  tuningOnlyPending = true;
  pending.active = false;
  beginTuningCommand();
  setStage(Stage::WaitTuningAck, "調整値をXIAOへ適用しています。");
  sendJsonResult(202, true, "調整値を送信しました。XIAOのACKを確認します。");
}

void apiLoggingPost() {
  if (!web.hasArg("enabled")) {
    sendJsonResult(400, false, "記録のON/OFF指定がありません。");
    return;
  }
  const String value = web.arg("enabled");
  if (value == "1" || value == "true") {
    if (!sdReady) {
      sendJsonResult(409, false, "SDカードを認識できていません。");
      return;
    }
    if (!beginSdLog()) {
      sendJsonResult(500, false, "SDログファイルを作成できません。");
      return;
    }
    sendJsonResult(200, true, "SD記録を開始しました。");
  } else if (value == "0" || value == "false") {
    stopSdLog();
    sendJsonResult(200, true, "SD記録を停止しました。");
  } else
    sendJsonResult(400, false, "記録のON/OFF指定が不正です。");
}

bool parseDoubleArgument(const char *name, double &value) {
  const String source = web.arg(name);
  if (!source.length())
    return false;
  char *end = nullptr;
  value = strtod(source.c_str(), &end);
  return end && *end == 0 && isfinite(value);
}

bool appendTrackJson(char *body, size_t capacity, size_t &used, const char *format, ...) {
  if (used >= capacity)
    return false;
  va_list arguments;
  va_start(arguments, format);
  const int written = vsnprintf(body + used, capacity - used, format, arguments);
  va_end(arguments);
  if (written < 0 || static_cast<size_t>(written) >= capacity - used)
    return false;
  used += static_cast<size_t>(written);
  return true;
}

void apiTrackGet() {
  static char body[9216];
  uint32_t since = 0;
  if (web.hasArg("since")) {
    char *end = nullptr;
    since = strtoul(web.arg("since").c_str(), &end, 10);
    if (!end || *end)
      since = 0;
  }
  const bool changed = !web.hasArg("since") || since != trackRevision;
  size_t used = 0;
  bool ok = appendTrackJson(body,
                            sizeof(body),
                            used,
                            "{\"revision\":%lu,\"changed\":%s,\"history\":[",
                            static_cast<unsigned long>(trackRevision),
                            changed ? "true" : "false");
  if (changed)
    for (uint16_t i = 0; i < trackHistoryCount && ok; ++i) {
      const TrackPoint &p = trackHistory[(trackHistoryHead + i) % kTrackHistoryCapacity];
      ok = appendTrackJson(body,
                           sizeof(body),
                           used,
                           "%s[%.7f,%.7f]",
                           i ? "," : "",
                           static_cast<double>(p.latitudeE7) / 1e7,
                           static_cast<double>(p.longitudeE7) / 1e7);
    }
  ok = ok && appendTrackJson(body, sizeof(body), used, "],\"waypoints\":");
  if (changed) {
    ok = ok && appendTrackJson(body, sizeof(body), used, "[");
    for (uint8_t i = 0; i < placedWaypointCount && ok; ++i)
      ok = appendTrackJson(body,
                           sizeof(body),
                           used,
                           "%s[%.7f,%.7f]",
                           i ? "," : "",
                           placedWaypoints[i].latitudeDeg,
                           placedWaypoints[i].longitudeDeg);
    ok = ok && appendTrackJson(body, sizeof(body), used, "]");
  } else
    ok = ok && appendTrackJson(body, sizeof(body), used, "null");
  const auto &fix = gnssRx.latest();
  const LinkCache control = cacheSnapshot();
  const bool courseValid = control.hasSnapshot && control.snapshot.gnssValid &&
                           ageMs(control.lastSnapshotUs, nowUs()) <= 3000UL &&
                           isfinite(control.snapshot.gnssCourseRad);
  ok = ok && appendTrackJson(body, sizeof(body), used, ",\"current\":");
  if (gnssFixFresh()) {
    if (courseValid)
      ok = ok && appendTrackJson(body,
                                 sizeof(body),
                                 used,
                                 "[%.7f,%.7f,%.5f]",
                                 fix.latitude,
                                 fix.longitude,
                                 control.snapshot.gnssCourseRad);
    else
      ok = ok && appendTrackJson(
                     body, sizeof(body), used, "[%.7f,%.7f,null]", fix.latitude, fix.longitude);
  } else
    ok = ok && appendTrackJson(body, sizeof(body), used, "null");
  ok = ok &&
       appendTrackJson(body, sizeof(body), used, " ,\"local_radius_m\":%.0f}", kTrackLocalRadiusM);
  if (!ok) {
    web.send(500, "application/json; charset=utf-8", "{\"error\":\"track_too_large\"}");
    return;
  }
  web.send(200, "application/json; charset=utf-8", body);
}

void apiDiagnosticsGet() {
  static char body[6144];
  const uint64_t currentUs = nowUs();
  const LinkCache cache = cacheSnapshot();
  const auto &fix = gnssRx.latest();
  const bool connected = linkConnected(cache), gnss = gnssFixFresh();
  const uint8_t safety = currentSafety(cache), reason = currentStopReason(cache);
  const bool pca = cache.hasActuators && cache.actuators.pcaReady;
  const bool imu = cache.hasSnapshot && cache.snapshot.imuValid;
  const bool tof = cache.hasSnapshot && cache.snapshot.tofValid;
  const bool controlGnss = cache.hasSnapshot && cache.snapshot.gnssValid;
  const bool power = cache.hasHealth && (cache.health.flags & 8u);
  const bool vesc = cache.hasHealth && (cache.health.flags & 16u);
  const char *linkDiagnosis =
      !cache.lastFrameUs
          ? "制御側から一度もUARTフレームを受信していません。制御側電源、制御側D7→通信側D1、通信側D0→制御側D6、GND共通を確認してください。"
          : (!connected
                 ? "以前は受信しましたが、最後のフレームから1秒を超えています。制御側の電源断・リセット・配線抜けを確認してください。"
                 : ((crcErrors || cobsErrors || lengthErrors)
                        ? "UARTは受信中ですが、受信エラーもあります。通信線とGNDを確認してください。"
                        : "制御側UARTリンクは正常です。"));
  size_t used = 0;
  bool ok = appendTrackJson(
      body,
      sizeof(body),
      used,
      "{\"uptime_ms\":%lu,\"communication\":{\"gnss_valid\":%s,\"gnss_age_ms\":%lu,\"satellites\":%u,\"sentences\":%lu,\"checksum_errors\":%lu,\"sd_ready\":%s,\"logging\":%s,\"wifi_clients\":%u},"
      "\"link\":{\"connected\":%s,\"ever_received\":%s,\"age_ms\":%lu,\"frames\":%lu,\"crc_errors\":%lu,\"cobs_errors\":%lu,\"length_errors\":%lu,\"diagnosis\":\"%s\"},"
      "\"control\":{\"snapshot\":%s,\"snapshot_age_ms\":%lu,\"actuators\":%s,\"actuators_age_ms\":%lu,\"safety\":%u,\"safety_name\":\"%s\",\"stop_reason\":%u,\"stop_reason_name\":\"%s\",\"pca\":%s,\"imu\":%s,\"tof\":%s,\"gnss\":%s,\"power\":%s,\"vesc\":%s},"
      "\"operation\":{\"stage\":\"%s\",\"waypoint\":%s,\"attitude\":%s},\"blockers\":[",
      static_cast<unsigned long>(millis()),
      gnss ? "true" : "false",
      static_cast<unsigned long>(ageMs(fix.lastValidFixUs, currentUs)),
      static_cast<unsigned>(fix.satellites),
      static_cast<unsigned long>(gnssSentences),
      static_cast<unsigned long>(gnssChecksumErrors),
      sdReady ? "true" : "false",
      sdLoggingEnabled ? "true" : "false",
      static_cast<unsigned>(WiFi.softAPgetStationNum()),
      connected ? "true" : "false",
      cache.lastFrameUs ? "true" : "false",
      static_cast<unsigned long>(ageMs(cache.lastFrameUs, currentUs)),
      static_cast<unsigned long>(cache.frames),
      static_cast<unsigned long>(crcErrors),
      static_cast<unsigned long>(cobsErrors),
      static_cast<unsigned long>(lengthErrors),
      linkDiagnosis,
      cache.hasSnapshot ? "true" : "false",
      static_cast<unsigned long>(ageMs(cache.lastSnapshotUs, currentUs)),
      cache.hasActuators ? "true" : "false",
      static_cast<unsigned long>(ageMs(cache.lastActuatorUs, currentUs)),
      static_cast<unsigned>(safety),
      safetyName(safety),
      static_cast<unsigned>(reason),
      stopReasonName(reason),
      pca ? "true" : "false",
      imu ? "true" : "false",
      tof ? "true" : "false",
      controlGnss ? "true" : "false",
      power ? "true" : "false",
      vesc ? "true" : "false",
      stageName(stage),
      operation.waypointEnabled ? "true" : "false",
      operation.attitudeEnabled ? "true" : "false");
  bool first = true;
  const auto addBlocker = [&](bool present, const char *text) {
    if (!present)
      return;
    ok = ok && appendTrackJson(body, sizeof(body), used, "%s\"%s\"", first ? "" : ",", text);
    first = false;
  };
  char reasonBlocker[72] = "";
  if (reason)
    snprintf(
        reasonBlocker, sizeof(reasonBlocker), "制御側の停止・制限理由: %s", stopReasonName(reason));
  addBlocker(!connected, "制御側XIAOとのUART通信がありません。");
  addBlocker(connected && !cache.hasSnapshot, "制御側から状態スナップショットが届いていません。");
  addBlocker(operation.waypointEnabled && !gnss,
             "ウェイポイント航行には通信側GNSSの有効な測位が必要です。");
  addBlocker(operation.waypointEnabled && connected && !controlGnss,
             "制御側がGNSS測位をまだ受信できていません。");
  addBlocker(operation.attitudeEnabled && !imu, "姿勢制御にはBNO08Xが必要です。");
  addBlocker(connected && !pca, "PCA9685が未認識のためサーボ出力できません。");
  addBlocker(operation.waypointEnabled && connected && !power,
             "推進には電源状態の取得が必要です。");
  addBlocker(operation.waypointEnabled && connected && !vesc, "推進にはVESCテレメトリが必要です。");
  addBlocker(reason != 0, reasonBlocker);
  if (first)
    ok = ok && appendTrackJson(
                   body, sizeof(body), used, "\"現在検出されている開始阻害要因はありません。\"");
  ok = ok && appendTrackJson(body, sizeof(body), used, "],\"events\":[");
  for (uint8_t i = 0; i < diagnosticEventCount && ok; ++i) {
    const uint8_t index =
        static_cast<uint8_t>((diagnosticEventHead + 16 - diagnosticEventCount + i) % 16);
    const DiagnosticEvent &event = diagnosticEvents[index];
    ok = appendTrackJson(body,
                         sizeof(body),
                         used,
                         "%s{\"age_ms\":%lu,\"text\":\"%s\"}",
                         i ? "," : "",
                         static_cast<unsigned long>(millis() - event.atMs),
                         event.text);
  }
  ok = ok && appendTrackJson(body, sizeof(body), used, "]}");
  if (!ok) {
    web.send(500, "application/json; charset=utf-8", "{\"error\":\"diagnostics_too_large\"}");
    return;
  }
  web.send(200, "application/json; charset=utf-8", body);
}

void apiTrackReset() {
  trackHistoryHead = 0;
  trackHistoryCount = 0;
  trackLastSampleMs = 0;
  ++trackRevision;
  sendJsonResult(200, true, "航跡をリセットしました。ウェイポイントは残しています。");
}

void apiTrackWaypointsPost() {
  if (stage != Stage::Idle && stage != Stage::Error) {
    sendJsonResult(409, false, "制御中はウェイポイントを変更できません。");
    return;
  }
  const String action = web.arg("action");
  if (action == "add") {
    double latitude = 0, longitude = 0;
    if (!parseDoubleArgument("lat", latitude) || !parseDoubleArgument("lon", longitude) ||
        latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0) {
      sendJsonResult(400, false, "緯度または経度が不正です。");
      return;
    }
    if (placedWaypointCount >= 16) {
      sendJsonResult(409, false, "ウェイポイントは16点までです。");
      return;
    }
    placedWaypoints[placedWaypointCount++] = {latitude, longitude};
    ++trackRevision;
    sendJsonResult(200, true, "ウェイポイントを追加しました。");
    return;
  }
  if (action == "delete") {
    char *end = nullptr;
    const long index = strtol(web.arg("index").c_str(), &end, 10);
    if (!end || *end || index < 0 || index >= placedWaypointCount) {
      sendJsonResult(400, false, "削除するウェイポイントが不正です。");
      return;
    }
    for (uint8_t i = static_cast<uint8_t>(index); i + 1 < placedWaypointCount; ++i)
      placedWaypoints[i] = placedWaypoints[i + 1];
    --placedWaypointCount;
    ++trackRevision;
    sendJsonResult(200, true, "ウェイポイントを削除しました。");
    return;
  }
  if (action == "clear") {
    placedWaypointCount = 0;
    ++trackRevision;
    sendJsonResult(200, true, "全ウェイポイントを削除しました。");
    return;
  }
  sendJsonResult(400, false, "ウェイポイント操作が不正です。");
}

void apiStart() {
  ManualValues values{};
  OperationConfig requested{};
  if (!parseManualValues(values)) {
    sendJsonResult(400, false, "手動出力値が不正です。");
    return;
  }
  if (!parseOperationConfig(requested)) {
    sendJsonResult(400, false, "制御設定または固定ウェイポイント選択が不正です。");
    return;
  }
  const LinkCache cache = cacheSnapshot();
  if (!linkConnected(cache)) {
    sendJsonResult(503, false, "XIAOと通信できていません。");
    return;
  }
  if (currentSafety(cache) == 4) {
    sendJsonResult(409, false, "緊急停止を解除してください。");
    return;
  }
  if (requested.attitudeEnabled &&
      (!cache.hasSnapshot || ageMs(cache.lastSnapshotUs, nowUs()) > kLinkFreshMs ||
       !cache.snapshot.imuValid)) {
    sendJsonResult(409, false, "姿勢補助に必要なBNO08Xが有効ではありません。");
    return;
  }
  if (stage != Stage::Idle && stage != Stage::Error) {
    sendJsonResult(409, false, "別の操作を処理中です。先に停止してください。");
    return;
  }
  manualValues = values;
  operation = requested;
  tuningOnlyPending = false;
  pending.active = false;
  lastManualMs = 0;
  portENTER_CRITICAL(&cacheMux);
  linkCache.sawFinalWaypoint = false;
  linkCache.latchedStopReason = 0;
  linkCache.output.stopReason = 0;
  linkCache.health.stopReason = 0;
  linkCache.snapshot.safetyReason = 0;
  portEXIT_CRITICAL(&cacheMux);
  setStage(Stage::EnsureDisarmed, "XIAOを停止状態にそろえています。");
  sendJsonResult(202, true, "開始手順を通信側XIAOで実行します。");
}

void apiValue() {
  ManualValues values{};
  if (!parseManualValues(values)) {
    sendJsonResult(400, false, "手動出力値が不正です。");
    return;
  }
  if (stage != Stage::Running) {
    sendJsonResult(409, false, "動作中ではありません。");
    return;
  }
  if (usesWaypoint()) {
    sendJsonResult(409, false, "ウェイポイント航行中は手動出力値を変更できません。");
    return;
  }
  manualValues = values;
  sendManualRefresh();
  sendJsonResult(202, true, "出力値を更新しました。");
}

void apiStop() {
  pending.active = false;
  sendSafety(boat::Type::Stop);
  sendManualOff();
  setStage(Stage::Stopping, "停止を確認しています。");
  sendJsonResult(202, true, "停止指令を送りました。");
}

void apiEstop() {
  pending.active = false;
  sendSafety(boat::Type::Estop);
  sendManualOff();
  setStage(Stage::Emergency, "緊急停止指令を送りました。");
  sendJsonResult(202, true, "緊急停止指令を送りました。");
}

void apiClearEstop() {
  const LinkCache cache = cacheSnapshot();
  if (currentSafety(cache) != 4) {
    sendJsonResult(409, false, "XIAOは緊急停止状態ではありません。");
    return;
  }
  sendSafety(boat::Type::ClearEstop);
  setStage(Stage::ClearingEmergency, "緊急停止の解除を確認しています。");
  sendJsonResult(202, true, "緊急停止解除指令を送りました。");
}

float sensorDisplayValue(float value) {
  return isfinite(value) ? value : 0.0f;
}

void apiSensors() {
  const LinkCache cache = cacheSnapshot();
  const uint64_t currentUs = nowUs();
  const auto &fix = gnssRx.latest();
  const bool connected = linkConnected(cache);
  const bool imuValid = cache.hasSnapshot && cache.snapshot.imuValid;
  const bool tofValid = cache.hasSnapshot && cache.snapshot.tofValid;
  const bool gnssValid = gnssFixFresh();
  const bool inaDetected = cache.hasHealth && (cache.health.flags & 64u);
  const bool inaValid = cache.hasIna && cache.ina.valid && cache.ina.ageUs <= 200000UL &&
                        ageMs(cache.lastInaStatusUs, currentUs) <= 1500UL;
  const bool vescValid = cache.hasVesc && cache.vesc.valid && cache.vesc.ageUs <= 300000UL &&
                         ageMs(cache.lastVescTelemetryUs, currentUs) <= 1500UL;
  char body[2200];
  snprintf(
      body,
      sizeof(body),
      "{\"connected\":%s,\"imu\":{\"valid\":%s,\"age_ms\":%lu,\"roll_deg\":%.2f,\"pitch_deg\":%.2f,\"yaw_deg\":%.2f,\"roll_rate_dps\":%.2f,\"pitch_rate_dps\":%.2f,\"yaw_rate_dps\":%.2f},"
      "\"tof\":{\"valid\":%s,\"age_ms\":%lu,\"distance_m\":%.3f},"
      "\"gnss\":{\"valid\":%s,\"age_ms\":%lu,\"latitude\":%.8f,\"longitude\":%.8f,\"speed_mps\":%.3f,\"satellites\":%u},"
      "\"ina\":{\"detected\":%s,\"valid\":%s,\"age_ms\":%lu,\"bus_v\":%.3f,\"shunt_v\":%.5f,\"current_a\":%.3f,\"power_w\":%.3f},"
      "\"vesc\":{\"valid\":%s,\"age_ms\":%lu,\"input_v\":%.3f,\"motor_current_a\":%.3f,\"input_current_a\":%.3f,\"duty\":%.4f,\"erpm\":%.1f,\"mos_temp_c\":%.1f,\"motor_temp_c\":%.1f,\"fault\":%u}}",
      connected ? "true" : "false",
      imuValid ? "true" : "false",
      static_cast<unsigned long>(cache.hasSnapshot ? cache.snapshot.imuAgeUs / 1000UL : UINT32_MAX),
      sensorDisplayValue(cache.snapshot.rollRad * 57.2957795f),
      sensorDisplayValue(cache.snapshot.pitchRad * 57.2957795f),
      sensorDisplayValue(cache.snapshot.yawRad * 57.2957795f),
      sensorDisplayValue(cache.snapshot.rollRateRadS * 57.2957795f),
      sensorDisplayValue(cache.snapshot.pitchRateRadS * 57.2957795f),
      sensorDisplayValue(cache.snapshot.yawRateRadS * 57.2957795f),
      tofValid ? "true" : "false",
      static_cast<unsigned long>(cache.hasSnapshot ? cache.snapshot.tofAgeUs / 1000UL : UINT32_MAX),
      sensorDisplayValue(cache.snapshot.tofFilteredM),
      gnssValid ? "true" : "false",
      static_cast<unsigned long>(ageMs(fix.lastValidFixUs, currentUs)),
      fix.latitude,
      fix.longitude,
      sensorDisplayValue(fix.speedMps),
      static_cast<unsigned>(fix.satellites),
      inaDetected ? "true" : "false",
      inaValid ? "true" : "false",
      static_cast<unsigned long>(cache.hasIna ? ageMs(cache.lastInaStatusUs, currentUs)
                                              : UINT32_MAX),
      sensorDisplayValue(cache.ina.busVoltageV),
      sensorDisplayValue(cache.ina.shuntVoltageV),
      sensorDisplayValue(cache.ina.currentA),
      sensorDisplayValue(cache.ina.powerW),
      vescValid ? "true" : "false",
      static_cast<unsigned long>(cache.hasVesc ? ageMs(cache.lastVescTelemetryUs, currentUs)
                                               : UINT32_MAX),
      sensorDisplayValue(cache.vesc.inputVoltageV),
      sensorDisplayValue(cache.vesc.motorCurrentA),
      sensorDisplayValue(cache.vesc.inputCurrentA),
      sensorDisplayValue(cache.vesc.duty),
      sensorDisplayValue(cache.vesc.erpm),
      sensorDisplayValue(cache.vesc.mosTempC),
      sensorDisplayValue(cache.vesc.motorTempC),
      static_cast<unsigned>(cache.vesc.fault));
  web.send(200, "application/json; charset=utf-8", body);
}

void apiStatus() {
  const LinkCache cache = cacheSnapshot();
  const bool connected = linkConnected(cache);
  const uint32_t linkAge = ageMs(cache.lastFrameUs, nowUs());
  const uint8_t safety = currentSafety(cache);
  const auto &fix = gnssRx.latest();
  const bool imuReady = cache.hasSnapshot && cache.snapshot.imuValid;
  const bool tofReady = cache.hasSnapshot && cache.snapshot.tofValid;
  const bool gnssCommunicationReady = gnssFixFresh();
  const bool gnssControlReady = cache.hasSnapshot && cache.snapshot.gnssValid;
  const bool powerReady = cache.hasHealth && (cache.health.flags & 8u);
  const bool vescReady = cache.hasHealth && (cache.health.flags & 16u);
  const bool inaReady = cache.hasHealth && (cache.health.flags & 64u);
  const fixed_waypoints::Point *target = fixed_waypoints::byIndex(operation.targetIndex);
  char targetName[8] = "?";
  if (operation.competitionCourse)
    strcpy(targetName, "COURSE");
  else if (operation.placedWaypoints)
    strcpy(targetName, "PLACED");
  else if (target) {
    targetName[0] = target->name;
    targetName[1] = '\0';
  }
  char body[3000];
  snprintf(
      body,
      sizeof(body),
      "{\"connected\":%s,\"ever_received\":%s,\"age_ms\":%lu,"
      "\"operation\":\"%s\",\"message\":\"%s\","
      "\"selection\":{\"waypoint\":%s,\"attitude\":%s,\"competition_course\":%s,\"placed_waypoints\":%s,\"target_index\":%u,"
      "\"target_name\":\"%s\",\"mode\":%u,\"mode_name\":\"%s\"},"
      "\"manual\":{\"enabled_mask\":%u,\"left\":%.3f,\"right\":%.3f,"
      "\"rear\":%.3f,\"propulsion\":%.3f},"
      "\"gnss\":{\"communication_valid\":%s,\"control_valid\":%u,\"age_ms\":%lu,"
      "\"latitude\":%.8f,\"longitude\":%.8f,\"speed_mps\":%.3f,"
      "\"satellites\":%u,\"sentences\":%lu,\"checksum_errors\":%lu},"
      "\"storage\":{\"sd_ready\":%s,\"logging_enabled\":%s,\"file\":\"%s\",\"records\":%lu,\"errors\":%lu},"
      "\"preflight\":{\"control_link\":%s,\"pca\":%s,\"imu\":%s,\"tof\":%s,"
      "\"gnss_communication\":%s,\"gnss_control\":%s,\"power\":%s,\"vesc\":%s,\"ina\":%s,"
      "\"waypoint_required\":%s,\"attitude_required\":%s},"
      "\"control\":{\"safety\":%u,\"safety_name\":\"%s\","
      "\"mode\":%u,\"stop_reason\":%u,\"stop_reason_name\":\"%s\","
      "\"waypoint_distance_m\":%.2f,\"active_waypoint\":%u,"
      "\"target_latitude\":%.8f,\"target_longitude\":%.8f},"
      "\"yaw\":{\"active\":%u,\"criteria_mask\":%u,\"vesc_low_erpm_warning\":%u,\"vesc_erpm\":%.1f},"
      "\"actuators\":{\"pca_ready\":%u,\"pwm_errors\":%lu,"
      "\"outputs_enabled\":%u,\"enabled_mask\":%u,\"left_us\":%u,"
      "\"right_us\":%u,\"rear_us\":%u,\"relay\":%u,"
      "\"target_duty\":%.3f,\"applied_duty\":%.3f},"
      "\"sensors\":{\"imu_valid\":%u,\"roll_rad\":%.4f,\"pitch_rad\":%.4f,"
      "\"yaw_rad\":%.4f,\"tof_valid\":%u,\"tof_m\":%.3f},"
      "\"link\":{\"frames\":%lu,\"crc_errors\":%lu,\"cobs_errors\":%lu,"
      "\"length_errors\":%lu}}",
      connected ? "true" : "false",
      cache.lastFrameUs ? "true" : "false",
      static_cast<unsigned long>(linkAge),
      stageName(stage),
      operationMessage,
      operation.waypointEnabled ? "true" : "false",
      operation.attitudeEnabled ? "true" : "false",
      operation.competitionCourse ? "true" : "false",
      operation.placedWaypoints ? "true" : "false",
      static_cast<unsigned>(operation.targetIndex),
      targetName,
      static_cast<unsigned>(operation.mode),
      modeName(operation.mode),
      static_cast<unsigned>(kAllManualMask),
      manualValues.left,
      manualValues.right,
      manualValues.rear,
      manualValues.propulsion,
      gnssFixFresh() ? "true" : "false",
      static_cast<unsigned>(cache.hasSnapshot ? cache.snapshot.gnssValid : 0),
      static_cast<unsigned long>(ageMs(fix.lastValidFixUs, nowUs())),
      fix.latitude,
      fix.longitude,
      fix.speedMps,
      static_cast<unsigned>(fix.satellites),
      static_cast<unsigned long>(gnssSentences),
      static_cast<unsigned long>(gnssChecksumErrors),
      sdReady ? "true" : "false",
      sdLoggingEnabled ? "true" : "false",
      sdLogPath,
      static_cast<unsigned long>(sdLogRecords),
      static_cast<unsigned long>(sdLogErrors),
      connected ? "true" : "false",
      (cache.hasActuators && cache.actuators.pcaReady) ? "true" : "false",
      imuReady ? "true" : "false",
      tofReady ? "true" : "false",
      gnssCommunicationReady ? "true" : "false",
      gnssControlReady ? "true" : "false",
      powerReady ? "true" : "false",
      vescReady ? "true" : "false",
      inaReady ? "true" : "false",
      operation.waypointEnabled ? "true" : "false",
      operation.attitudeEnabled ? "true" : "false",
      static_cast<unsigned>(safety),
      safetyName(safety),
      static_cast<unsigned>(cache.hasSnapshot ? cache.snapshot.mode : 0),
      static_cast<unsigned>(currentStopReason(cache)),
      stopReasonName(currentStopReason(cache)),
      cache.hasSnapshot ? cache.snapshot.waypointDistanceM : 0.0f,
      static_cast<unsigned>(cache.hasSnapshot ? cache.snapshot.activeWaypoint : 0),
      cache.hasSnapshot ? cache.snapshot.targetWaypointLatitudeDeg : 0.0,
      cache.hasSnapshot ? cache.snapshot.targetWaypointLongitudeDeg : 0.0,
      static_cast<unsigned>(cache.hasSnapshot ? cache.snapshot.yawControlActive : 0),
      static_cast<unsigned>(cache.hasSnapshot ? cache.snapshot.yawCriteriaMask : 0),
      static_cast<unsigned>(cache.hasSnapshot ? cache.snapshot.vescLowErpmWarning : 0),
      cache.hasSnapshot && isfinite(cache.snapshot.vescErpm) ? cache.snapshot.vescErpm : 0.0f,
      static_cast<unsigned>(cache.hasActuators ? cache.actuators.pcaReady : 0),
      static_cast<unsigned long>(cache.hasActuators ? cache.actuators.pwmErrors : 0),
      static_cast<unsigned>(cache.hasActuators ? cache.actuators.outputsEnabled : 0),
      static_cast<unsigned>(cache.hasActuators ? cache.actuators.enabledMask : 0),
      static_cast<unsigned>(cache.hasActuators ? cache.actuators.leftPulseUs : 0),
      static_cast<unsigned>(cache.hasActuators ? cache.actuators.rightPulseUs : 0),
      static_cast<unsigned>(cache.hasActuators ? cache.actuators.rearPulseUs : 0),
      static_cast<unsigned>(cache.hasActuators ? cache.actuators.motorRelayEnabled : 0),
      cache.hasActuators ? cache.actuators.targetDuty : 0.0f,
      cache.hasActuators ? cache.actuators.appliedDuty : 0.0f,
      static_cast<unsigned>(cache.hasSnapshot ? cache.snapshot.imuValid : 0),
      cache.hasSnapshot && isfinite(cache.snapshot.rollRad) ? cache.snapshot.rollRad : 0.0f,
      cache.hasSnapshot && isfinite(cache.snapshot.pitchRad) ? cache.snapshot.pitchRad : 0.0f,
      cache.hasSnapshot && isfinite(cache.snapshot.yawRad) ? cache.snapshot.yawRad : 0.0f,
      static_cast<unsigned>(cache.hasSnapshot ? cache.snapshot.tofValid : 0),
      cache.hasSnapshot && isfinite(cache.snapshot.tofFilteredM) ? cache.snapshot.tofFilteredM
                                                                 : 0.0f,
      static_cast<unsigned long>(cache.frames),
      static_cast<unsigned long>(crcErrors),
      static_cast<unsigned long>(cobsErrors),
      static_cast<unsigned long>(lengthErrors));
  web.send(200, "application/json; charset=utf-8", body);
}

void startWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(kApSsid, kApPassword);
  web.on("/", HTTP_GET, [] {
    web.send(200, "text/html; charset=utf-8", productionPageJapanese);
  });
  web.on("/track", HTTP_GET, [] {
    web.send(200, "text/html; charset=utf-8", productionTrackPageJapanese);
  });
  web.on("/diagnostics", HTTP_GET, [] {
    web.send(200, "text/html; charset=utf-8", productionDiagnosticsPageJapanese);
  });
  web.on("/sensors", HTTP_GET, [] {
    web.send(200, "text/html; charset=utf-8", productionSensorsPageJapanese);
  });
  web.on("/api/status", HTTP_GET, apiStatus);
  web.on("/api/sensors", HTTP_GET, apiSensors);
  web.on("/api/diagnostics", HTTP_GET, apiDiagnosticsGet);
  web.on("/api/track", HTTP_GET, apiTrackGet);
  web.on("/api/track/reset", HTTP_POST, apiTrackReset);
  web.on("/api/track/waypoints", HTTP_POST, apiTrackWaypointsPost);
  web.on("/api/tuning", HTTP_GET, apiTuningGet);
  web.on("/api/tuning", HTTP_POST, apiTuningPost);
  web.on("/api/logging", HTTP_POST, apiLoggingPost);
  web.on("/api/start", HTTP_POST, apiStart);
  web.on("/api/value", HTTP_POST, apiValue);
  web.on("/api/stop", HTTP_POST, apiStop);
  web.on("/api/estop", HTTP_POST, apiEstop);
  web.on("/api/clear-estop", HTTP_POST, apiClearEstop);
  web.onNotFound([] {
    web.send(404, "application/json", "{\"error\":\"not_found\"}");
  });
  web.begin();
}

void printStatus() {
  const LinkCache cache = cacheSnapshot();
  const bool connected = linkConnected(cache);
  const uint8_t safety = currentSafety(cache);
  const auto &fix = gnssRx.latest();
  const fixed_waypoints::Point *target = fixed_waypoints::byIndex(operation.targetIndex);
  Serial.printf("LINK=%s age=%lu frames=%lu CONTROL=%s COMM=%s MODE=%s WP=%c "
                "GNSS=%s sat=%u SD=%s REASON=%s AP=http://%s/ UART_ERR=%lu/%lu/%lu\n",
                connected ? "OK" : "WAIT",
                static_cast<unsigned long>(ageMs(cache.lastFrameUs, nowUs())),
                static_cast<unsigned long>(cache.frames),
                safetyName(safety),
                stageName(stage),
                modeName(operation.mode),
                target ? target->name : '?',
                gnssFixFresh() ? "FIX" : "WAIT",
                fix.satellites,
                sdReady ? "OK" : "WAIT",
                stopReasonName(currentStopReason(cache)),
                WiFi.softAPIP().toString().c_str(),
                static_cast<unsigned long>(crcErrors),
                static_cast<unsigned long>(cobsErrors),
                static_cast<unsigned long>(lengthErrors));
}

} // namespace

void setup() {
  Serial.begin(115200);
  bootId = esp_random();
  if (!bootId)
    bootId = 1;
  frameSequence = esp_random();
  safetyCommandId = esp_random();
  initializePersistentCounters();

  gnssUart.setRxBufferSize(kGnssUartRxBufferBytes);
  gnssRx.begin(gnssUart);

  SPI.begin(kSdSckPin, kSdMisoPin, kSdMosiPin, kSdCsPin);
  sdReady = SD.begin(kSdCsPin, SPI, kSdSpiHz);

  controlUart.setRxBufferSize(16384);
  controlUart.setTimeout(2);
  controlUart.begin(kControlUartBaud, SERIAL_8N1, kControlUartRxPin, kControlUartTxPin);
  xTaskCreatePinnedToCore(controlRxTask, "ControlRx", 6144, nullptr, 3, &rxTaskHandle, 1);
  startWeb();

  Serial.printf("%s %s GNSS_RX=%d GNSS_TX=%d CONTROL_RX=%d CONTROL_TX=%d "
                "SD_SCK=%d SD_MISO=%d SD_MOSI=%d SD_CS=%d SD=%s baud=%lu AP=%s URL=http://%s/\n",
                kFirmwareName,
                kFirmwareVersion,
                kGnssRxPin,
                kGnssTxPin,
                kControlUartRxPin,
                kControlUartTxPin,
                kSdSckPin,
                kSdMisoPin,
                kSdMosiPin,
                kSdCsPin,
                sdReady ? "OK" : "NOT_READY",
                static_cast<unsigned long>(kControlUartBaud),
                kApSsid,
                WiFi.softAPIP().toString().c_str());
}

void loop() {
  web.handleClient();
  serviceGnss();
  serviceTrack();
  serviceDiagnostics();
  sendHeartbeat();
  serviceOperation();
  serviceSdLog();
  if (millis() - lastStatusLogMs >= kStatusLogPeriodMs) {
    lastStatusLogMs = millis();
    printStatus();
  }
  delay(1);
}
