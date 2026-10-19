#include "../../day1/communication/src/main.cpp"
#include <production_control.h>
#include <cassert>
#include <cstring>

SerialConsole Serial;
SDClass SD;
SPIClass SPI;
WiFiStub WiFi;
int SerialConsole::printf(const char *, ...) { return 0; }

production_control::SensorInput nominal(uint64_t now = 1000000) {
  production_control::SensorInput input{};
  input.safety = production_control::AuthoritativeSafety::Running;
  input.nowUs = now;
  input.heartbeat = input.imuValid = input.tofValid = input.gnssValid = true;
  input.powerValid = input.vescValid = true;
  input.heartbeatUs = input.imuUs = input.tofUs = input.gnssUs = now;
  input.powerUs = input.vescUs = now;
  input.tofM = .45f;
  input.busVoltageV = 12.0f;
  input.currentA = 3.0f;
  input.vescErpm = 1000.0f;
  input.groundSpeedMps = 1.0f;
  return input;
}

int main() {
  web.args = {{"waypoint", "1"}, {"attitude", "0"}, {"target", "course"}};
  OperationConfig selected{};
  assert(parseOperationConfig(selected));
  assert(selected.competitionCourse && selected.mode == boat::ControlWaypointOnly);
  operation = selected;
  beginWaypointCommand();
  assert(pending.active && pending.type == boat::Type::WaypointSet);
  boat::WaypointSetPayload packet{};
  assert(pending.length == sizeof(packet));
  std::memcpy(&packet, pending.payload, sizeof(packet));
  assert(packet.action == 1 && packet.count == 11);
  assert(packet.canonicalCrc == boat::canonicalCrc(&packet, offsetof(boat::WaypointSetPayload, canonicalCrc)));
  const char expected[] = "ABCDEFGHCBA";
  production_control::Waypoint route[11]{};
  for (uint8_t i = 0; i < packet.count; ++i) {
    const auto &point = fixed_waypoints::kPoints[fixed_waypoints::kCompetitionRouteIndices[i]];
    assert(point.name == expected[i]);
    assert(packet.points[i].latitudeDeg == point.latitudeDeg);
    assert(packet.points[i].longitudeDeg == point.longitudeDeg);
    route[i].northM = static_cast<float>((point.latitudeDeg - 35.45) * 111320.0);
    route[i].eastM = static_cast<float>((point.longitudeDeg - 136.07) * 90000.0);
  }
  production_control::Controller controller;
  assert(controller.setWaypoints(route, packet.count, 1, production_control::AuthoritativeSafety::Disarmed).ack == production_control::Ack::Accepted);
  assert(controller.setMode(production_control::ControlMode::WaypointOnly, 2, production_control::AuthoritativeSafety::Disarmed).ack == production_control::Ack::Accepted);
  auto input = nominal();
  input.northM = route[0].northM - 100;
  input.eastM = route[0].eastM;
  assert(controller.step(input).safetyRequest == production_control::SafetyRequest::None);
  for (uint8_t i = 0; i < packet.count; ++i) {
    input = nominal(1100000 + i * 100000);
    input.northM = route[i].northM;
    input.eastM = route[i].eastM;
    const auto output = controller.step(input);
    assert(output.waypointReached);
    if (i + 1 < packet.count) {
      assert(output.activeWaypoint == i + 1);
      assert(output.safetyRequest == production_control::SafetyRequest::None);
    } else {
      assert(output.reason == production_control::StopReason::FinalWaypoint);
      assert(output.safetyRequest == production_control::SafetyRequest::Disarm);
      assert(output.propulsion == 0 && !output.physicalGate);
    }
  }
  web.args["target"] = "2";
  assert(parseOperationConfig(selected) && !selected.competitionCourse && selected.targetIndex == 2);
  operation = selected;
  beginWaypointCommand();
  std::memcpy(&packet, pending.payload, sizeof(packet));
  assert(packet.action == 1 && packet.count == 1);
  assert(packet.points[0].latitudeDeg == fixed_waypoints::kPoints[2].latitudeDeg);
  web.args["target"] = "invalid";
  assert(!parseOperationConfig(selected));
}
