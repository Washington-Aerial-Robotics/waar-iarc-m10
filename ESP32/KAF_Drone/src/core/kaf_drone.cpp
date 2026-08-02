#include "kaf_drone.h"

drone_state kafenv;

peripheral kaf_reset() {
  DPRINTF( "[K] Resetting KAF Drone\n" );
  kafenv.info.deviceID = 'U';
  kafenv.info.flightMode = 0;
  kafenv.info.triggerLock = 0;
  kafenv.info.actuation = false;
  kafenv.info.version = 0x20260527;
  kafenv.info.battery = 100.0F;
  FPFILL0( i, kafenv.state.x.f );
  FPFILL0( i, kafenv.state.v.f );
  FPFILL0( i, kafenv.state.q.f );
  FPFILL0( i, kafenv.state.w.f );
  FPFILL0( i, kafenv.cmd.motors );
  FPFILL0( i, kafenv.cmd.setpoints );
  kafenv.cal.anglealpha    = 0.15F;
  kafenv.cal.positionalpha = 9.53F;
  kafenv.cal.gravitation   = 9.81F;
  ITRVEC3( i ) kafenv.cal.accelfilt[i] = { 1, 0, 0 };
  ITRVEC3( i ) kafenv.cal.gyrofilt[i]  = { 1, 0, 0 };
  ITRVEC3( i ) kafenv.cal.magfilt[i]  = { 1, 0, 0 };
  kafenv.cal.accelfilt[2].ofst = kafenv.cal.gravitation;
  for( unsigned char i = 0; i < sizeof( kafenv.cal.sensefilt ) / sizeof( coordinate ); i++ ) {
    kafenv.cal.sensefilt[i] = { 1, 0, 0 };
  }
  kafenv.cal.xpid    = { 1, 0.01F, 0 };
  kafenv.cal.vpid    = { 1, 0.01F, 0 };
  kafenv.cal.apid    = { 1, 0.01F, 0 };
  kafenv.cal.qpid    = { 1, 0.01F, 0 };
  kafenv.cal.wpid[0] = { 1, 0.01F, 0 };
  kafenv.cal.wpid[1] = { 1, 0.01F, 0 };
  kafenv.cal.wpid[2] = { 1, 0.01F, 0 };
  return { "kafenv", sizeof( kafenv ), sizeof( kafenv ), &kafenv, [](){ kaf_reset(); }, NULLPTR };
}