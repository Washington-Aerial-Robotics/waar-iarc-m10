#include "kaf_drone.h"

drone_state kafenv;

memory kaf_reset() {
  DPRINTF( "[K] Resetting KAF Drone\n" );
  kafenv.info.deviceID = 'U';
  kafenv.info.flightMode = 0;
  kafenv.info.trigger = MAXBYTE;
  kafenv.info.actuation = false;
  kafenv.info.version = 0x20260416;
  kafenv.info.battery = 100.0F;
  FPFILL0( i, kafenv.state.x.f );
  FPFILL0( i, kafenv.state.v.f );
  FPFILL0( i, kafenv.state.q.f );
  FPFILL0( i, kafenv.state.w.f );
  FPFILL0( i, kafenv.cmd.motors );
  FPFILL0( i, kafenv.cmd.setpoints );
  kafenv.cal.anglealpha    = 0.33F;
  kafenv.cal.positionalpha = 15.3F;
  kafenv.cal.gravitation   = 0.3F;
  kafenv.cal.xpid    = { 1, 0.01F, 0 };
  kafenv.cal.vpid    = { 1, 0.01F, 0 };
  kafenv.cal.qpid    = { 1, 0.01F, 0 };
  kafenv.cal.wpid[0] = { 1, 0.01F, 0 };
  kafenv.cal.wpid[1] = { 1, 0.01F, 0 };
  kafenv.cal.wpid[2] = { 1, 0.01F, 0 };
  ITRVEC3( i ) kafenv.cal.accelfilt[i] = { 1, 0, 0 };
  ITRVEC3( i ) kafenv.cal.gyrofilt[i]  = { 1, 0, 0 };
  ITRVEC3( i ) kafenv.cal.magfilt[i]  = { 1, 0, 0 };
  for( unsigned char i = 0; i < sizeof( kafenv.cal.sensefilt ) / sizeof( coordinate ); i++ ) {
    kafenv.cal.sensefilt[i] = { 1, 0, 0 };
  }
  memory mempage = { "kafenv", sizeof( kafenv ), &kafenv };
  return mempage;
}