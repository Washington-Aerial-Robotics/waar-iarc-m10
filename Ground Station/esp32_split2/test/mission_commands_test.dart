import 'dart:typed_data';
import 'package:flutter_test/flutter_test.dart';
import 'package:esp32_split/controllers/drone_controller.dart';
import 'package:esp32_split/protocol/drone_protocol.dart';
import 'telemetry_requests_test.dart' show TestClient;

Uint8List info({
  bool armed = false,
  int mode = 0,
  int slot = 0,
  int qual = 0,
  int square = 0,
  int sensors = 15,
}) {
  final bytes = Uint8List(20);
  bytes[0] = 0x55;
  bytes[3] = armed ? 1 : 0;
  bytes[12] = mode;
  bytes[13] = slot;
  bytes[14] = qual;
  bytes[15] = 3;
  bytes[16] = square;
  bytes[17] = sensors;
  return bytes;
}

void main() {
  late TestClient client;
  late DroneController controller;
  Future<void> report(Uint8List data) async {
    final request = controller.requestInfo();
    client.reply(0x03, data);
    expect(await request, isTrue);
  }

  setUp(() async {
    client = TestClient();
    controller = DroneController(client);
    await controller.connect('test', 70);
  });
  tearDown(() async {
    controller.dispose();
    await client.incoming.close();
    client.dispose();
  });
  for (var mode = 0; mode < 4; mode++) {
    test('KILL is transmitted in autonomy mode $mode', () async {
      await report(info(mode: mode));
      controller.kill();
      expect(client.sent.last[2], 0x63);
    });
  }
  test('20-byte INFO padding and combined receipt preserve framing', () async {
    final request = controller.requestInfo();
    final payload = info(mode: 3);
    payload[18] = 0xAA;
    payload[19] = 0xBB;
    client.incoming.add(
      Uint8List.fromList([
        0x47,
        0x55,
        0x03,
        client.sent.last[3],
        ...payload,
        0x47,
        0x55,
        0x27,
        99,
      ]),
    );
    expect(await request, isTrue);
    expect(controller.telemetry.autonomyMode, 3);
    expect(
      controller.log.any((line) => line.contains('Unsupported reply')),
      isFalse,
    );
  });
  test(
    'extended INFO decodes all fields and preserves false armed when merged',
    () {
      final telemetry = DroneTelemetry.fromInfoReply(
        info(armed: true, mode: 3, slot: 2, qual: 4, square: 6),
      )!;
      expect(telemetry.armed, isTrue);
      expect(telemetry.autonomyMode, 3);
      expect(telemetry.formationSlot, 2);
      expect(telemetry.qualState, 4);
      expect(telemetry.qualRevolutions, 3);
      expect(telemetry.squareState, 6);
      expect(telemetry.sensorStatus, 15);
      expect(
        telemetry.mergedWith(const DroneTelemetry(armed: false)).armed,
        isFalse,
      );
      expect(DroneTelemetry.fromInfoReply(Uint8List(12))!.autonomyMode, isNull);
    },
  );
  test(
    'receipt alone does not confirm; fresh INFO confirms mode and slot',
    () async {
      await report(info());
      final mode = controller.setAutonomyMode(1);
      expect(client.sent.last.sublist(2), [0x64, client.sent.last[3], 1]);
      client.reply(0x24, Uint8List(0));
      expect(controller.missionCommandPending, isTrue);
      await report(info(mode: 1));
      expect(await mode, isTrue);
      final slot = controller.setFormationSlot(2);
      await report(info(mode: 1, slot: 2));
      expect(await slot, isTrue);
    },
  );
  test(
    'armed configuration and missing sensors block; LAND ABORT bypass gates',
    () async {
      await report(info(armed: true, mode: 1, sensors: 14));
      expect(await controller.setAutonomyMode(3), isFalse);
      expect(await controller.setFormationSlot(2), isFalse);
      expect(await controller.sendQualCommand(1), isFalse);
      final land = controller.sendQualCommand(4);
      expect(client.sent.last[2], 0x66);
      expect(client.sent.last[4], 4);
      await report(info(armed: true, mode: 1, qual: 5, sensors: 0));
      expect(await land, isTrue);
      final abort = controller.sendSquareCommand(
        3,
      ); // transmitted even in wrong mode
      expect(client.sent.last[2], 0x67);
      controller.kill();
      expect(client.sent.last[2], 0x63);
      expect(await abort, isFalse);
    },
  );
  test('square start gated; unchanged INFO rejects at deadline', () async {
    await report(info(mode: 3, sensors: 14));
    expect(await controller.sendSquareCommand(1), isFalse);
    await report(info(mode: 3));
    final start = controller.sendSquareCommand(1);
    client.reply(0x27, Uint8List(0));
    await report(info(mode: 3));
    expect(await start, isFalse);
    expect(controller.missionStatus, contains('rejected/unconfirmed'));
  });
  test('INFO timer polls at 2 Hz and stops on disconnect', () async {
    await Future<void>.delayed(const Duration(milliseconds: 550));
    expect(client.sent.last[2], 0x43);
    client.reply(0x03, info());
    final count = client.sent.length;
    await Future<void>.delayed(const Duration(milliseconds: 500));
    expect(client.sent.length, count + 1);
    await controller.disconnect();
    final stopped = client.sent.length;
    await Future<void>.delayed(const Duration(milliseconds: 550));
    expect(client.sent.length, stopped);
  });
}
