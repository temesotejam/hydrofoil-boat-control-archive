// GNSS input processing and result acknowledgement.
void stop(); void printP1TxSnapshot();
void sendCommandAck(const boat::Frame&f,uint8_t disposition,uint16_t reason){boat::CommandPayload c{};if(f.header.length==sizeof(c))memcpy(&c,f.payload,sizeof(c));boat::CommandAckPayload ack{c.commandId,c.commandType,disposition,(uint8_t)safety,(uint8_t)kDryRunActuators,nowUs(),nowUs(),reason,0};linkSend(boat::Type::CommandAck,(uint8_t*)&ack,sizeof(ack));}

float wrapGnssHeading(float value){while(value>PI)value-=2.0f*PI;while(value<-PI)value+=2.0f*PI;return value;}
void resetGnssHeading(){gnssHeading={};}
void pushGnssHeading(float northM,float eastM,uint64_t timestampUs){
  GnssHeadingSample& sample=gnssHeading.samples[gnssHeading.next];
  sample.northM=northM;sample.eastM=eastM;sample.timestampUs=timestampUs;
  gnssHeading.next=(gnssHeading.next+1)%kGnssHeadingWindowSamples;
  if(gnssHeading.count<kGnssHeadingWindowSamples)++gnssHeading.count;
  gnssHeading.valid=false;
  if(gnssHeading.count<kGnssHeadingMinSamples)return;
  const uint8_t oldest=gnssHeading.count==kGnssHeadingWindowSamples?gnssHeading.next:0;
  const uint8_t newest=(gnssHeading.next+kGnssHeadingWindowSamples-1)%kGnssHeadingWindowSamples;
  const GnssHeadingSample& first=gnssHeading.samples[oldest];
  const GnssHeadingSample& last=gnssHeading.samples[newest];
  const float north=last.northM-first.northM,east=last.eastM-first.eastM;
  const float seconds=timestampUs>first.timestampUs?(timestampUs-first.timestampUs)*1.0e-6f:0.0f;
  if(seconds<=0.0f)return;
  const float course=atan2f(east,north);
  float rate=0.0f;
  if(gnssHeading.previousCourseValid&&timestampUs>gnssHeading.previousCourseUs){
    const float dt=(timestampUs-gnssHeading.previousCourseUs)*1.0e-6f;
    if(dt>0.0f)rate=constrain(wrapGnssHeading(course-gnssHeading.previousCourseRad)/dt,-1.5f,1.5f);
  }
  gnssHeading.courseRad=course;gnssHeading.yawRateRadS=rate;gnssHeading.timestampUs=timestampUs;
  gnssHeading.previousCourseRad=course;gnssHeading.previousCourseUs=timestampUs;gnssHeading.previousCourseValid=true;gnssHeading.valid=true;
}

void processNav(const boat::Frame& f) {
  boat::GnssNavPayload nav{};
  if (f.header.length != sizeof(nav)) { ++navCrcErrors; return; }
  memcpy(&nav, f.payload, sizeof(nav));
  navigation.latitudeDeg=nav.latitudeE7/1e7; navigation.longitudeDeg=nav.longitudeE7/1e7; navigation.speedMps=nav.speedMmPerSec/1000.0f; navigation.courseRad=(nav.courseE5Deg/100000.0f)*PI/180.0f; navigation.timestampUs=nowUs();
  uint32_t flags = 0;
  if (nav.canonicalCrc != boat::canonicalCrc(&nav, offsetof(boat::GnssNavPayload, canonicalCrc))) { ++navCrcErrors; flags |= 8; }
  const bool navValid = (nav.flags & boat::NavFixValid) && !(flags & 8);
  navigation.valid=navValid;
  controlState.updateGnss(nowUs(), nav.latitudeE7 / 1e7,
                          nav.longitudeE7 / 1e7,
                          nav.speedMmPerSec / 1000.0f,
                          (nav.courseE5Deg / 100000.0f) * PI / 180.0f,
                          navValid);
  const bool duplicate = lastNavSequence && nav.navSequence == lastNavSequence;
  if (duplicate) flags |= 2;
  if (lastNavSequence && nav.navSequence != lastNavSequence + 1 && !duplicate) { flags |= 4; ++navSequenceGaps; }
  lastNavSequence = nav.navSequence;
  if (navValid && !navOriginSet) { navOriginSet = true; navOriginLatE7 = nav.latitudeE7; navOriginLonE7 = nav.longitudeE7; syncCompetitionWaypoints(); }
  boat::GnssProcessResultPayload result{};
  result.navSequence = nav.navSequence; result.fixSequence = nav.fixSequence; result.navCanonicalCrc = nav.canonicalCrc; result.flags = flags; result.controlReceivedUs = nowUs();
  result.originLatitudeE7 = navOriginLatE7; result.originLongitudeE7 = navOriginLonE7; result.speedMmPerSec = nav.speedMmPerSec;
  result.courseMilliRad = (int32_t)lround((nav.courseE5Deg / 100000.0) * PI / 180.0 * 1000.0);
  if (navOriginSet) { result.flags |= 16; result.northMm = (int32_t)lround((nav.latitudeE7 - navOriginLatE7) * 11.132); result.eastMm = (int32_t)lround((nav.longitudeE7 - navOriginLonE7) * 11.132 * cos(navOriginLatE7 / 1e7 * PI / 180.0)); navNorthM=result.northMm*.001f;navEastM=result.eastMm*.001f; }
  if(navValid&&navOriginSet&&!duplicate)pushGnssHeading(navNorthM,navEastM,navigation.timestampUs);else if(!navValid)resetGnssHeading();
  if (navValid) result.flags |= 1;
  result.errorCode = (flags & 8) ? 1 : 0; result.safetyState = (uint16_t)safety; result.dryRun = kDryRunActuators; result.controlSentUs = nowUs();
  result.canonicalCrc = boat::canonicalCrc(&result, offsetof(boat::GnssProcessResultPayload, canonicalCrc));
  linkSend(boat::Type::GnssProcessResult, (uint8_t*)&result, sizeof(result)); ++gnssResultTx;
}
