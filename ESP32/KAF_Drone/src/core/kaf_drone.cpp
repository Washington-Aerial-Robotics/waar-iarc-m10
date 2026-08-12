#include "kaf_drone.h"

drone_state kafenv;

peripheral kaf_reset() {
  DPRINTF( "[K] Resetting KAF Drone\n" );
  kafenv.info.deviceID = 'U';
  kafenv.info.flightMode = 0;
  kafenv.info.triggerLock = 0;
  kafenv.info.actuation = false;
  //bumped from 0x20260527: removing wifi from the persistent-EEPROM layout (see periph_wifi.cpp) shifts
  //every persistent field registered after it. This invalidates any EEPROM blob written under the old
  //layout - kafenv/mpu9250/commander/pidtuner - so it fails validation and gets reset to defaults
  //instead of silently misapplying misaligned bytes (e.g. old wifi credential bytes read as commander
  //trajectory data, or old commander bytes read as PID gains).
  //bumped again from 0x20260808: appending kafenv.cal.hoverThrust grew kafenv's persisted length, which
  //would otherwise make firmware_handlePersistents() read one float past the end of an old EEPROM blob.
  kafenv.info.version = 0x20260810;
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
  //defaults to 0 (no feedforward at all - thrust comes only from the +-0.9-bounded apid PID term) rather
  //than a guessed nonzero value, since no hover-throttle fraction has been measured for this airframe yet.
  //Must be set explicitly (webserver Calibration page, "Hover FF" field) before ACCEL_SETPOINT_MODE/
  //POS_SETPOINT_MODE will produce meaningful sustained thrust.
  kafenv.cal.hoverThrust = 0;
  return { "kafenv", sizeof( kafenv ), sizeof( kafenv ), &kafenv, [](){ kaf_reset(); }, NULLPTR };
}