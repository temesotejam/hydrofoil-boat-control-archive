#include <cassert>
#include <cmath>
#include <cstdio>
#define private public
#include "production_control.h"
#undef private
#include "fixed_waypoints.h"
using namespace production_control;
SensorInput nominal(){SensorInput x{};x.nowUs=1000000;x.heartbeatUs=x.imuUs=x.gnssUs=x.tofUs=x.powerUs=x.vescUs=x.nowUs;x.imuValid=x.gnssValid=x.tofValid=x.powerValid=x.vescValid=true;x.busVoltageV=12;x.heartbeat=true;x.tofM=.45f;x.vescErpm=2000;x.safety=AuthoritativeSafety::Running;return x;}
int main(){
 Controller c;Config cfg{};c.setConfig(cfg);
 Waypoint p[11];auto o=fixed_waypoints::kPoints[0];
 double scale=111320*std::cos(o.latitudeDeg*3.141592653589793/180);
 for(int i=0;i<11;++i){auto v=fixed_waypoints::kPoints[fixed_waypoints::kCompetitionRouteIndices[i]];p[i]={(float)((v.latitudeDeg-o.latitudeDeg)*111320),(float)((v.longitudeDeg-o.longitudeDeg)*scale)};}
 c.setWaypoints(p,11,0,AuthoritativeSafety::Disarmed,true);assert(c.competitionGateCount_==168);
 c.setMode(ControlMode::WaypointOnly,1,AuthoritativeSafety::Disarmed);
 auto x=nominal();x.imuValid=false;auto out=c.step(x);assert(out.safetyRequest==SafetyRequest::Fault);
 x=nominal();x.imuUs=x.nowUs-100001;out=c.step(x);assert(out.safetyRequest==SafetyRequest::Fault);
 x=nominal();out=c.step(x);assert(out.safetyRequest==SafetyRequest::None);
 // No control-output LPF: changing heading immediately changes uYaw by PD formula.
 c.setMode(ControlMode::HeadingHold,2,AuthoritativeSafety::Disarmed);c.setHeading(.4f,2);c.setManual(ManualCommand(0,0,0,.5f,ManualAll),1000000);
 x=nominal();x.groundSpeedMps=0;x.yawRad=0;x.yawRateRadS=0;
 out=c.step(x);assert(std::fabs(out.uYaw-cfg.kpYaw*.4f)<1e-5f);
 x.nowUs+=20000;x.heartbeatUs=x.imuUs=x.gnssUs=x.tofUs=x.powerUs=x.vescUs=x.nowUs;x.yawRad=.2f;
 out=c.step(x);assert(std::fabs(out.uYaw-cfg.kpYaw*.2f)<1e-5f);
 std::puts("168 gates; IMU invalid/stale/fresh; immediate yaw output passed");
}
