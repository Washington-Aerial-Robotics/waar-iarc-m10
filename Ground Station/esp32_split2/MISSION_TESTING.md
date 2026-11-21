# Mission controls and props-off testing

The app now exposes the four mission SET messages on a Mission tab. Firmware is unchanged.

## Compatibility and confirmation

TCP parsing targets **20-byte INFO payloads** (24 bytes including header), consuming padding bytes 18–19. Legacy 12-byte INFO cannot be auto-detected in this unframed stream. Use matching v0.9.0+ firmware. The standalone payload decoder still supports legacy data for offline use.

A separate 500 ms timer polls INFO while connected, skipping ticks if another telemetry request is outstanding. SET receipt replies 0x24–0x27 do not confirm acceptance. A SET cancels the old telemetry request; confirmation requires matching INFO requested after SET. After 1.5 seconds without the expected state, status is rejected/unconfirmed: missing telemetry does not prove rejection.

Expected outcomes: mode/slot equals requested value; QUAL LAUNCH reaches CLIMB/HOVER, BEGIN ORBIT reaches ORBIT/POST_HOLD, HOLD reaches POST_HOLD, LAND/ABORT reaches LANDING/FINISH. Square START reaches CLIMB/LEG1–4, LAND/ABORT reaches LANDING/FINISH. Mission outcomes also require matching autonomy mode. Already-satisfied states count as observed outcomes, not proof a new command executed.

INFO older than 1.5 seconds inhibits normal mission actions. Mode/slot require disarmed INFO. LAUNCH/START require matching mode and all sensor bits. These bits do not guarantee valid position; firmware remains authoritative.

LAND/ABORT always remain available while connected, bypassing app sensor/mode/pending gates. **The actual firmware still requires the corresponding mission mode before it handles LAND/ABORT.** KILL remains available; firmware currently handles it by restarting the ESP32.

Mission commands stop manual motor streaming. Manual ARM and Console Test Control require fresh MANUAL-mode INFO. Autonomous LAUNCH/START do not require pressing manual ARM first. New voice mission commands are not included.

## Hardware checklist — pending

1. Remove propellers, flash matching firmware, connect in Console, then open Mission.
2. Confirm armed, battery, mode, slot, states, revolutions, and sensor chips against firmware output.
3. Change mode/slot disarmed; verify INFO confirmation. Armed configuration must be disabled.
4. Remove GPS readiness: LAUNCH/START must be blocked. Check BARO/IMU/MAG gates too.
5. With valid position, verify QUAL and SQUARE state transitions independently using serial logs.
6. Test LAND/ABORT from applicable states with missing sensors and a pending command. Confirm actual onboard outcome, not just receipt.
7. Test KILL in each mode and inspect motor outputs during/after restart.
8. Test stale INFO, Wi-Fi loss/reconnect, Android, and iPhone before flight.

## Sensor-board check — pending

The local commander computes IMU/MAG bits through `peripheral_mpu9250Working()` and `peripheral_mpu9250MagWorking()`. If the board uses ICM-20948, the firmware team must verify/adapt those accessors. The app only displays reported bits. Physical bench qualification and ICM-20948 verification have not been performed.
