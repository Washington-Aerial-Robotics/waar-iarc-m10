import 'dart:typed_data';

class DroneComms {
  static const int headerLen = 4;

  static const int comCmd = 0x40 ;
  static const int comFwd = 0x80;

  static const int comSuccess = 0x3A;
  static const int comFailure = 0x3B;
  static const int comAcknowledged = 0x3C;

  static const int comPing = 0x40;
  static const int comPong = 0x00;

  static const int comSetCtrlMode = 0x4C;
  static const int comSetPosCmd = 0x4F;
  static const int comSetMotorCmd = 0x40 | 16;

  static const int comRequestWifi = 0x61;
  static const int comReplyWifi = 0x17;

  static const int comSetWifi = 0x58;
  static const int comReplySetWifi = 0x18;

  static const int comKill = 0x40 | 35;

  static const int comRequestPos = 0x63;
  static const int comReplyPos = 0x23;

  static const int comRequestAtt = 0x65;
  static const int comReplyAtt = 0x25;

  static const int comRequestStateEst = 0x61;
  static const int comReplyStateEst = 0x21;

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