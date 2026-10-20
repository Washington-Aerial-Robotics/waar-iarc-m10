# Kent firmware integration

## Target and connection

App-only integration against local `origin/Firmware-Kent` commit `416ac12` (Version 20260527). Kent firmware is unchanged. Defaults: SSID `KAF_Quadcopter_Drone`, empty password, IP `192.168.1.240`, raw TCP port `70`. Persisted firmware settings may override these defaults.

## Implemented behavior

- Header remains destination ID, source ID, type, message ID. App ID is G, default target is A. Disconnect before changing target.
- Ping `0x40` expects `0x00`; Wi-Fi request `0x61` expects a 20-byte IP payload in `0x21`; Get State `0x42` expects three float32 position values plus mode byte in `0x02`.
- Success/failure/acknowledged/invalid replies are `0x3C/0x3D/0x3E/0x3F`. Requests match reply destination, source, message ID, and expected type. Timeout is two seconds. One request is outstanding at a time; no automatic command retries.
- ARM resets controls, requests actuation disabled (`0x4A`, byte 0), selects motor-setpoint mode (`0x4C`, mode 4, count 4, four zero floats), then enables actuation (`0x4A`, byte 255). Each step waits for success before continuing. The loop starts only after all three replies.
- The existing bench motor mixer remains in the app. The 4 Hz loop now sends setpoints (`0x4F`, count 4, four little-endian floats) because Kent motor mode copies setpoints to motor outputs. Outstanding replies skip updates. An update failure stops local transmission and requires reconnecting; it does not prove remote motors stopped.
- DISARM interrupts local control, requests actuation disabled, then selects null mode with count 0, waiting for replies. Logs explicitly report unconfirmed failures.
- KILL interrupts local operations and sends `0x63` immediately. Kent handles it by restarting the ESP32. No execution confirmation or latched hardware-stop guarantee is claimed.
- Console Test Control is disabled to prevent bypassing the mode/actuation sequence. Use Remote for bench controls. Voice hover remains unavailable; other supported phrases adjust the existing bench inputs.

## Stream handling

The app buffers split replies and extracts combined replies for enabled reply types. Unsupported types cause a protocol error and socket disconnect rather than guessed framing. State-estimate replies (`0x04`, 48 bytes) are also recognized. This is a limited decoder, not support for every Kent message or forwarding variant.

Kent's Wi-Fi receiver still treats available socket bytes as a received message. TCP can split or combine commands; app-side reply buffering alone cannot fix firmware-side message assembly. Sequential acknowledged control reduces overlap but does not guarantee firmware framing. This needs coordinated firmware validation/work.

## Confirmation limits

Success is protocol acceptance, not a measured motor-state or flight-state readback. Mode/actuation transitions are not yet independently verified through info telemetry. No heartbeat-based onboard failsafe, autonomous/manual ownership handover, or stabilized joystick mapping has been added. Motor mixing is retained only as an explicit bench interface, not an asserted competition flight interface.

## Bench verification

Remove propellers. Join the ESP32 network, connect to port 70, and test PING, Get IP, and Get State first. Verify IDs against the actual flashed build. Check ARM ordering, DISARM, KILL/restart outputs, rejection, missing replies, Wi-Fi loss, and reconnect without automatic rearming using firmware logs. Validate on Android and iPhone. Automated tests cover payload structure, reply chunk assembly, control cancellation, DISARM sequencing, link reset, and layouts; they do not qualify physical flight behavior.

## Next steps with firmware team

Agree on stabilized setpoint units and reference frame, override/autonomy handover, measured mode/actuation readback, command-age failsafe, and true emergency-stop semantics. Then map voice/joystick intentions to those interfaces. Keep high-rate stabilization onboard.
