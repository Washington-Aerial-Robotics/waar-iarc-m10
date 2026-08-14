import 'dart:typed_data';

class DroneComms {
  static const int headerLen = 4;

  static const int comCmd = 0x40 ;
  static const int comFwd = 0x80;

  // Status codes (COM_SUCCESS / COM_FAILURE / COM_ACKNOWLEDGED / COM_INVALID
  // in communication.h — these are shared replies to any COM_SET_* command).
  static const int comSuccess = 0x3C;
  static const int comFailure = 0x3D;
  static const int comAcknowledged = 0x3E;
  static const int comInvalid = 0x3F;

  static const int comPing = 0x40;
  static const int comPong = 0x00;

  static const int comSetCtrlMode = 0x4C;
  static const int comSetPosCmd = 0x4F;
  static const int comSetMotorCmd = 0x40 | 16;
  static const int comSetActuation = 0x40 | 10;

  // Low 3 bits of the firmware's flightMode byte (flight.h DEFAULT_MODES_MASK)
  // - selects which flight_step() case governs kafenv.cmd.motors each tick.
  static const int flightModeNull = 0;
  static const int flightModeActuation = 3; // pass-through: leaves cmd.motors alone, so COM_SET_MOTORS packets reach the ESCs
  static const int flightModeMotorSetpoint = 4; // overwrites cmd.motors from cmd.setpoints every tick - NOT what COM_SET_MOTORS needs

  // Upper 5 bits (commander.h CMD_MODE_MASK) - the commander state machine.
  // CMD_NULL_MODE skips commander_step()'s storage-read/failsafe logic entirely,
  // which is what a manual bench/motor test wants.
  static const int cmdModeNull = 16;

  static const int comRequestWifi = 0x61;
  static const int comReplyWifi = 0x21;

  static const int comSetWifi = 0x62;

  static const int comKill = 0x40 | 35;
  static const int comSetAutonomyMode = 0x40 | 36;
  static const int comSetFormationSlot = 0x40 | 37;
  static const int comSetQualCommand = 0x40 | 38;
  static const int comSetSquareCommand = 0x40 | 39;

  // kafenv.info.autonomyMode values (commander.h) - orthogonal to flightMode.
  static const int autonomyManual = 0;
  static const int autonomyQualification = 1;
  static const int autonomyMineSearch = 2;
  static const int autonomySquareTest = 3;

  // QUALCMD_* (commander.h) - COM_SET_QUALCOMMAND payload values.
  static const int qualCmdLaunch = 1;
  static const int qualCmdBeginOrbit = 2;
  static const int qualCmdHold = 3;
  static const int qualCmdLand = 4;
  static const int qualCmdAbort = 5;

  // QUAL_* (commander.h) - kafenv.info.qualState values, for display.
  static const int qualStateBoot = 0;
  static const int qualStateClimbToFormation = 1;
  static const int qualStateHoverHold = 2;
  static const int qualStateOrbit = 3;
  static const int qualStatePostOrbitHold = 4;
  static const int qualStateLanding = 5;
  static const int qualStateFinish = 6;

  // SQUARECMD_* (commander.h) - COM_SET_SQUARECOMMAND payload values.
  static const int squareCmdStart = 1;
  static const int squareCmdLand = 2;
  static const int squareCmdAbort = 3;

  // SQUARE_* (commander.h) - kafenv.info.squareState values, for display.
  static const int squareStateBoot = 0;
  static const int squareStateClimb = 1;
  static const int squareStateLeg1 = 2;
  static const int squareStateLeg2 = 3;
  static const int squareStateLeg3 = 4;
  static const int squareStateLeg4 = 5;
  static const int squareStateLanding = 6;
  static const int squareStateFinish = 7;

  // Maps to COM_REQUEST_STATE / COM_REPLY_STATE — the "dronestate" reply,
  // which carries position + flight status. There is no dedicated
  // position-only message in the firmware.
  static const int comRequestPos = 0x42;
  static const int comReplyPos = 0x02;

  // Maps to COM_REQUEST_STEST / COM_REPLY_STEST — the full state-estimate
  // reply (position, velocity, attitude, angular rate). There is no
  // separate attitude-only message; request this for attitude too.
  static const int comRequestStateEst = 0x44;
  static const int comReplyStateEst = 0x04;

  // Maps to COM_REQUEST_INFO / COM_REPLY_INFO — the "droneinfo" reply,
  // which carries device identification, firmware version, and battery.
  static const int comRequestInfo = 0x43;
  static const int comReplyInfo = 0x03;

  static const int nullMode = 0x00;
  static const int calibrationMode = 0x01;
  static const int motorSetpointMode = 0x02;
  static const int posSetpointMode = 0x03;
  static const int trajectoryMode = 0x04;
}

class DronePacket {
  const DronePacket({
    required this.toId,
    required this.fromId,
    required this.messageType,
    required this.messageId,
    required this.payload,
  });

  final int toId;
  final int fromId;
  final int messageType;
  final int messageId;
  final Uint8List payload;

  bool get isCommand => (messageType & DroneComms.comCmd) != 0;
  bool get isForwarded => (messageType & DroneComms.comFwd) != 0;

  @override
  String toString() {
    return 'DronePacket(to=0x${toId.toRadixString(16)}, '
        'from=0x${fromId.toRadixString(16)}, '
        'type=0x${messageType.toRadixString(16)}, '
        'msgId=0x${messageId.toRadixString(16)}, '
        'payloadLen=${payload.length})';
  }
}

class DronePacketBuilder {
  DronePacketBuilder({
    required this.fromId,
    required this.defaultToId,
  });

  final int fromId;
  final int defaultToId;

  int _nextMessageId = 5;

  int nextMessageId() {
    final id = _nextMessageId & 0xFF;
    _nextMessageId = (_nextMessageId + 1) & 0xFF;
    return id;
  }

  Uint8List headerOnly({
    required int messageType,
    int? toId,
    int? messageId,
  }) {
    return build(
      toId: toId ?? defaultToId,
      messageType: messageType,
      payload: Uint8List(0),
      messageId: messageId ?? nextMessageId(),
    );
  }

  int thrustByte(double v) {
    return (v.clamp(0.0, 1.0) * 255).round();
  }

  int signedStickByte(double v) {
    final signed = (v.clamp(-1.0, 1.0) * 127).round();
    return signed & 0xFF;
  }

  Uint8List motor4Floats({
  required double m0,
  required double m1,
  required double m2,
  required double m3,
  int? toId,
  int? messageId,
}) {
  final bd = ByteData(16);

  bd.setFloat32(0, m0.clamp(0.0, 1.0), Endian.little);
  bd.setFloat32(4, m1.clamp(0.0, 1.0), Endian.little);
  bd.setFloat32(8, m2.clamp(0.0, 1.0), Endian.little);
  bd.setFloat32(12, m3.clamp(0.0, 1.0), Endian.little);

  return build(
    toId: toId ?? defaultToId,
    messageType: DroneComms.comSetMotorCmd,
    payload: bd.buffer.asUint8List(),
    messageId: messageId ?? nextMessageId(),
  );
}

  /// Builds a COM_SET_FLIGHTMODE packet. Payload matches the firmware's
  /// packed `flightmode.h` struct: [flightMode: uint8][commandLength: uint8],
  /// with commandLength=0 so no setpoint floats are required in the payload.
  Uint8List flightMode({
    required int cmdMode,
    required int mode,
    int? toId,
    int? messageId,
  }) {
    return build(
      toId: toId ?? defaultToId,
      messageType: DroneComms.comSetCtrlMode,
      payload: Uint8List.fromList([(cmdMode | mode) & 0xFF, 0]),
      messageId: messageId ?? nextMessageId(),
    );
  }

  /// Builds a COM_SET_ACTUATION packet. Firmware checks the byte against
  /// MAXBYTE (0xFF) exactly, so anything else (including 0x00) disarms.
  Uint8List actuation({
    required bool armed,
    int? toId,
    int? messageId,
  }) {
    return singleByte(
      messageType: DroneComms.comSetActuation,
      value: armed ? 0xFF : 0x00,
      toId: toId,
      messageId: messageId,
    );
  }

  /// Builds a COM_SET_AUTONOMYMODE packet. Firmware rejects this while
  /// armed and never itself arms/moves anything on receipt - mode
  /// selection and arming are always two separate operator actions.
  Uint8List autonomyMode({
    required int mode,
    int? toId,
    int? messageId,
  }) {
    return singleByte(
      messageType: DroneComms.comSetAutonomyMode,
      value: mode,
      toId: toId,
      messageId: messageId,
    );
  }

  /// Builds a COM_SET_FORMATIONSLOT packet (0-3). Firmware rejects this
  /// while armed.
  Uint8List formationSlot({
    required int slot,
    int? toId,
    int? messageId,
  }) {
    return singleByte(
      messageType: DroneComms.comSetFormationSlot,
      value: slot,
      toId: toId,
      messageId: messageId,
    );
  }

  /// Builds a COM_SET_QUALCOMMAND packet (DroneComms.qualCmd*). Only
  /// acted on while autonomyMode == autonomyQualification.
  Uint8List qualCommand({
    required int command,
    int? toId,
    int? messageId,
  }) {
    return singleByte(
      messageType: DroneComms.comSetQualCommand,
      value: command,
      toId: toId,
      messageId: messageId,
    );
  }

  /// Builds a COM_SET_SQUARECOMMAND packet (DroneComms.squareCmd*). Only
  /// acted on while autonomyMode == autonomySquareTest.
  Uint8List squareCommand({
    required int command,
    int? toId,
    int? messageId,
  }) {
    return singleByte(
      messageType: DroneComms.comSetSquareCommand,
      value: command,
      toId: toId,
      messageId: messageId,
    );
  }

  Uint8List singleByte({
    required int messageType,
    required int value,
    int? toId,
    int? messageId,
  }) {
    return build(
      toId: toId ?? defaultToId,
      messageType: messageType,
      payload: Uint8List.fromList([value & 0xFF]),
      messageId: messageId ?? nextMessageId(),
    );
  }

  Uint8List coordinate4f({
    required int messageType,
    required double a,
    required double b,
    required double c,
    double d = 0.0,
    int? toId,
    int? messageId,
  }) {
    final bd = ByteData(16);
    bd.setFloat32(0, a, Endian.little);
    bd.setFloat32(4, b, Endian.little);
    bd.setFloat32(8, c, Endian.little);
    bd.setFloat32(12, d, Endian.little);

    return build(
      toId: toId ?? defaultToId,
      messageType: messageType,
      payload: bd.buffer.asUint8List(),
      messageId: messageId ?? nextMessageId(),
    );
  }

  Uint8List build({
    required int toId,
    required int messageType,
    required Uint8List payload,
    required int messageId,
  }) {
    final out = Uint8List(DroneComms.headerLen + payload.length);

    out[0] = toId & 0xFF;
    out[1] = fromId & 0xFF;
    out[2] = messageType & 0xFF;
    out[3] = messageId & 0xFF;

    out.setRange(DroneComms.headerLen, out.length, payload);
    return out;
  }

  static DronePacket? tryParse(Uint8List bytes) {
    if (bytes.length < DroneComms.headerLen) return null;

    return DronePacket(
      toId: bytes[0],
      fromId: bytes[1],
      messageType: bytes[2],
      messageId: bytes[3],
      payload: Uint8List.sublistView(bytes, DroneComms.headerLen),
    );
  }
}

/// A 3-float vector, matching the firmware's packed `coordinate` union
/// (12 bytes: x,y,z as little-endian float32).
class Coordinate3 {
  const Coordinate3(this.x, this.y, this.z);

  final double x;
  final double y;
  final double z;

  static Coordinate3 fromBytes(ByteData bd, int offset) {
    return Coordinate3(
      bd.getFloat32(offset, Endian.little),
      bd.getFloat32(offset + 4, Endian.little),
      bd.getFloat32(offset + 8, Endian.little),
    );
  }

  @override
  String toString() {
    return '(${x.toStringAsFixed(2)}, '
        '${y.toStringAsFixed(2)}, '
        '${z.toStringAsFixed(2)})';
  }
}

/// Decoded telemetry from the drone, accumulated from whichever reply
/// packets have arrived so far. Fields are null until a reply that reports
/// them has been received at least once.
class DroneTelemetry {
  const DroneTelemetry({
    this.position,
    this.velocity,
    this.attitude,
    this.angularRate,
    this.flightMode,
    this.batteryPercent,
    this.deviceId,
    this.firmwareVersion,
    this.armed,
    this.autonomyMode,
    this.formationSlot,
    this.qualState,
    this.qualRevolutions,
    this.squareState,
  });

  final Coordinate3? position;
  final Coordinate3? velocity;

  // Firmware calls this block "q" but it is 3 floats, not a full
  // quaternion: yaw lives in .z, with .x/.y as body-frame components
  // (see communication.h's stateestimate comment).
  final Coordinate3? attitude;
  final Coordinate3? angularRate;

  final int? flightMode;
  final double? batteryPercent;
  final int? deviceId;
  final int? firmwareVersion;
  // kafenv.info.actuation - true while motor actuation is enabled
  // (armed), independent of which flightMode is selected.
  final bool? armed;

  // AUTONOMY_MANUAL/QUALIFICATION/MINE_SEARCH (see commander.h) - which
  // autonomy behavior the drone is running, phone-selected via
  // COM_SET_AUTONOMYMODE.
  final int? autonomyMode;
  // 0-3, phone-set via COM_SET_FORMATIONSLOT before a Qualification launch.
  final int? formationSlot;
  // QUAL_BOOT..QUAL_FINISH (see commander.h) - only meaningful when
  // autonomyMode == QUALIFICATION.
  final int? qualState;
  final int? qualRevolutions;
  // SQUARE_BOOT..SQUARE_FINISH (see commander.h) - only meaningful when
  // autonomyMode == SQUARE_TEST.
  final int? squareState;

  /// Returns a copy with any non-null fields from [update] overlaid on top
  /// of this telemetry, so partial replies (e.g. position-only) don't wipe
  /// out fields reported by a different reply type.
  DroneTelemetry mergedWith(DroneTelemetry update) {
    return DroneTelemetry(
      position: update.position ?? position,
      velocity: update.velocity ?? velocity,
      attitude: update.attitude ?? attitude,
      angularRate: update.angularRate ?? angularRate,
      flightMode: update.flightMode ?? flightMode,
      batteryPercent: update.batteryPercent ?? batteryPercent,
      deviceId: update.deviceId ?? deviceId,
      firmwareVersion: update.firmwareVersion ?? firmwareVersion,
      armed: update.armed ?? armed,
      autonomyMode: update.autonomyMode ?? autonomyMode,
      formationSlot: update.formationSlot ?? formationSlot,
      qualState: update.qualState ?? qualState,
      qualRevolutions: update.qualRevolutions ?? qualRevolutions,
      squareState: update.squareState ?? squareState,
    );
  }

  /// Decodes a COM_REPLY_STATE payload ("dronestate"): position + flight
  /// status. Wire layout (packed, little-endian):
  /// [x,y,z: float32][status: uint8] = 13 bytes.
  static DroneTelemetry? fromStateReply(Uint8List payload) {
    if (payload.length < 13) return null;

    final bd = ByteData.sublistView(payload);
    return DroneTelemetry(
      position: Coordinate3.fromBytes(bd, 0),
      flightMode: payload[12],
    );
  }

  /// Decodes a COM_REPLY_STEST payload ("stateestimate"): position,
  /// velocity, attitude, angular rate. Wire layout: four back-to-back
  /// coordinate blocks (x, v, q, w), 12 bytes each = 48 bytes total.
  static DroneTelemetry? fromStateEstimateReply(Uint8List payload) {
    if (payload.length < 48) return null;

    final bd = ByteData.sublistView(payload);
    return DroneTelemetry(
      position: Coordinate3.fromBytes(bd, 0),
      velocity: Coordinate3.fromBytes(bd, 12),
      attitude: Coordinate3.fromBytes(bd, 24),
      angularRate: Coordinate3.fromBytes(bd, 36),
    );
  }

  /// Decodes a COM_REPLY_INFO payload ("droneinfo"): identification,
  /// firmware version, battery, and autonomy/qualification/square-test
  /// status. Wire layout: [deviceID,flightMode,triggerLock,actuation:
  /// uint8 x4][version: uint32][battery: float32][autonomyMode,
  /// formationSlot,qualState,qualRevolutions,squareState: uint8 x5] = 17
  /// bytes. Bytes past the first 12 are read defensively (payload.length
  /// checks) so this keeps working against older firmware that sends
  /// fewer of them.
  static DroneTelemetry? fromInfoReply(Uint8List payload) {
    if (payload.length < 12) return null;

    final bd = ByteData.sublistView(payload);
    return DroneTelemetry(
      deviceId: payload[0],
      flightMode: payload[1],
      armed: payload[3] != 0,
      firmwareVersion: bd.getUint32(4, Endian.little),
      batteryPercent: bd.getFloat32(8, Endian.little),
      autonomyMode: payload.length >= 13 ? payload[12] : null,
      formationSlot: payload.length >= 14 ? payload[13] : null,
      qualState: payload.length >= 15 ? payload[14] : null,
      qualRevolutions: payload.length >= 16 ? payload[15] : null,
      squareState: payload.length >= 17 ? payload[16] : null,
    );
  }

  @override
  String toString() {
    final parts = <String>[
      if (position != null) 'pos=$position',
      if (velocity != null) 'vel=$velocity',
      if (attitude != null) 'att=$attitude',
      if (angularRate != null) 'w=$angularRate',
      if (flightMode != null) 'mode=$flightMode',
      if (batteryPercent != null)
        'batt=${batteryPercent!.toStringAsFixed(1)}%',
      if (deviceId != null)
        'id=0x${deviceId!.toRadixString(16).padLeft(2, '0')}',
      if (firmwareVersion != null) 'fw=$firmwareVersion',
    ];

    return parts.isEmpty ? '(no telemetry yet)' : parts.join('  ');
  }
}