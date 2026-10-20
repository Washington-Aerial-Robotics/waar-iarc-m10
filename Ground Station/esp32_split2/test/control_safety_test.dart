import 'dart:async';
import 'dart:typed_data';
import 'package:flutter_test/flutter_test.dart';
import 'package:esp32_split/controllers/drone_controller.dart';
import 'package:esp32_split/protocol/drone_protocol.dart';
import 'package:esp32_split/services/tcp_client.dart';

class FakeClient extends TcpClient {
  bool connected = false;
  final changes = StreamController<bool>.broadcast(sync: true);
  final sent = <List<int>>[];
  bool acknowledge = true;
  final replies = StreamController<Uint8List>.broadcast(sync: true);
  @override
  Stream<Uint8List> get packets => replies.stream;
  @override
  bool get isConnected => connected;
  @override
  Stream<bool> get connectionChanges => changes.stream;
  @override
  Future<void> connect(
    String ip,
    int port, {
    Duration timeout = const Duration(seconds: 5),
  }) async {
    connected = true;
  }

  @override
  void sendBytes(List<int> bytes) {
    sent.add(List.of(bytes));
    if (acknowledge && bytes[2] != DroneComms.comKill) {
      replies.add(
        Uint8List.fromList([
          bytes[1],
          bytes[0],
          DroneComms.comSuccess,
          bytes[3],
        ]),
      );
    }
  }

  void loseLink() {
    connected = false;
    changes.add(false);
  }

  @override
  void dispose() {
    changes.close();
    replies.close();
    super.dispose();
  }
}

void main() {
  test('missing ARM reply times out without starting motor transmission', () async {
    final client = FakeClient()..acknowledge = false;
    final controller = DroneController(client);
    await controller.connect('test', 70);
    await controller.arm();
    expect(controller.isControlLoopRunning, isFalse);
    expect(client.sent.length, 1);
    expect(controller.log.any((line) => line.contains('ARM failed')), isTrue);
    controller.dispose();
    client.dispose();
  });

  test('Kent decoder assembles split and combined replies', () {
    final decoder = KentReplyDecoder();
    expect(decoder.add([0x47, 0x41]), isEmpty);
    final replies = decoder.add([0x00, 5, 0x47, 0x41, 0x3C, 6]);
    expect(replies.map((r) => r.messageId), [5, 6]);
    expect(() => decoder.add([0x47, 0x41, 0xFE, 0]), throwsFormatException);
  });
  test('Kent motor-mode payload includes count and float values', () {
    final builder = DronePacketBuilder(fromId: 0x47, defaultToId: 0x41);
    final packet = builder.setpoints([
      0,
      0,
      0,
      0,
    ], mode: DroneComms.motorSetpointMode);
    expect(packet[2], 0x4C);
    expect(packet.sublist(4, 6), [4, 4]);
    expect(packet.length, 22);
  });
  test(
    'KILL interrupts pending ARM without later enabling actuation',
    () async {
      final client = FakeClient()..acknowledge = false;
      final controller = DroneController(client);
      await controller.connect('test', 70);
      final arming = controller.arm();
      expect(controller.isControlLoopRunning, isFalse);
      controller.kill();
      await arming;
      expect(client.sent.map((p) => p[2]), [0x4A, 0x63]);
      expect(controller.isControlLoopRunning, isFalse);
      controller.dispose();
      client.dispose();
    },
  );
  test(
    'DISARM interrupts pending ARM and completes disable sequence',
    () async {
      final client = FakeClient()..acknowledge = false;
      final controller = DroneController(client);
      await controller.connect('test', 70);
      final arming = controller.arm();
      client.acknowledge = true;
      await controller.disarm();
      await arming;
      expect(controller.isControlLoopRunning, isFalse);
      expect(client.sent.any((p) => p[2] == 0x4A && p[4] == 255), isFalse);
      controller.dispose();
      client.dispose();
    },
  );

  test('disconnect stops controls and reconnect requires ARM', () async {
    final client = FakeClient();
    final controller = DroneController(client);
    await controller.connect('test', 1234);
    controller.setSticks(t: 0.8, p: 0.4);
    await controller.arm();
    client.loseLink();
    expect(controller.status, LinkStatus.disconnected);
    expect(controller.isControlLoopRunning, isFalse);
    expect(controller.throttle, 0);
    expect(controller.pitch, 0);
    await controller.connect('test', 1234);
    expect(controller.isControlLoopRunning, isFalse);
    controller.dispose();
    client.dispose();
  });
  test('DISARM disables actuation followed by Kent null mode', () async {
    final client = FakeClient();
    final controller = DroneController(client);
    await controller.connect('test', 1234);
    await controller.arm();
    client.sent.clear();
    await controller.disarm();
    expect(client.sent.length, 2);
    expect(client.sent.first[2], 0x4A);
    expect(client.sent.first.sublist(4), everyElement(0));
    expect(client.sent.last[2], 0x4C);
    expect(client.sent.last[4], DroneComms.nullMode);
    expect(controller.isControlLoopRunning, isFalse);
    controller.dispose();
    client.dispose();
  });
  test(
    'hover does not substitute a fixed throttle for altitude hold',
    () async {
      final client = FakeClient();
      final controller = DroneController(client);
      await controller.connect('test', 1234);
      await controller.arm();
      controller.setSticks(t: 0.2);
      controller.voiceHover();
      expect(controller.throttle, 0.2);
      controller.kill();
      expect(client.sent.last[2], 0x63);
      controller.dispose();
      client.dispose();
    },
  );
}
