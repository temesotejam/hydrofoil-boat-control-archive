#include <cassert>
#include <cmath>
#include "production_control.h"
#include "competition_actuators.h"
using namespace production_control;
int main(){
 Config cfg{};assert(cfg.autoPropulsion==1.0f);cfg.slewPerStep=1.0f;
 Controller c;c.setConfig(cfg);Waypoint p{};p.northM=1000;
 c.setWaypoints(&p,1,0,AuthoritativeSafety::Disarmed);c.setMode(ControlMode::WaypointOnly,1,AuthoritativeSafety::Disarmed);
 SensorInput in{};in.safety=AuthoritativeSafety::Running;in.nowUs=1000000;
 in.heartbeat=in.imuValid=in.tofValid=in.gnssValid=in.powerValid=in.vescValid=true;
 in.heartbeatUs=in.imuUs=in.tofUs=in.gnssUs=in.powerUs=in.vescUs=in.nowUs;
 in.tofM=.45f;in.busVoltageV=12;in.vescErpm=2000;
 auto out=c.step(in);assert(out.propulsionPrelimit==1.0f);assert(out.propulsion==1.0f);
 for(float maximum:{1.0f,.5f}){
  DutyRamp ramp(maximum,2.0f,.35f);ramp.setTarget(out.propulsion*maximum);
  float value=0;for(int i=0;i<110;++i)value=ramp.step(.02f);
  assert(std::fabs(value-maximum)<1e-5f);
 }
 in.busVoltageV=8;out=c.step(in);assert(out.safetyRequest==SafetyRequest::Fault&&out.propulsion==0);
 in.busVoltageV=12;in.safety=AuthoritativeSafety::EStop;out=c.step(in);assert(out.propulsion==0);
}
