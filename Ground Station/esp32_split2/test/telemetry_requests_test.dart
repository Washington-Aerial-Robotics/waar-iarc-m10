import 'dart:async';
import 'dart:typed_data';
import 'package:flutter_test/flutter_test.dart';
import 'package:esp32_split/controllers/drone_controller.dart';
import 'package:esp32_split/services/tcp_client.dart';

class TestClient extends TcpClient {
  final incoming = StreamController<Uint8List>.broadcast(sync: true);
  final sent = <List<int>>[];
  @override
  bool get isConnected => true;
  @override
  Stream<Uint8List> get packets => incoming.stream;
  @override
  Future<void> connect(
    String ip,
    int port, {
    Duration timeout = const Duration(seconds: 5),
  }) async {}
  @override
  void sendBytes(List<int> bytes) => sent.add(List.of(bytes));
  void reply(int type, Uint8List payload, {int? id}) {
    incoming.add(
      Uint8List.fromList([0x47, 0x55, type, id ?? sent.last[3], ...payload]),
    );
  }
}

void main() {
  late TestClient client;
  late DroneController controller;
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
  test('state request buffers split reply and checks message ID', () async {
    final result = controller.requestState();
    expect(client.sent.last[2], 0x42);
    final payload = ByteData(13)
      ..setFloat32(0, 1.5, Endian.little)
      ..setUint8(12, 6);
    client.reply(0x02, payload.buffer.asUint8List(), id: 99);
    expect(controller.telemetryRequestPending, isTrue);
    final reply = [
      0x47,
      0x55,
      0x02,
      client.sent.last[3],
      ...payload.buffer.asUint8List(),
    ];
    client.incoming.add(Uint8List.fromList(reply.sublist(0, 7)));
    expect(controller.telemetryRequestPending, isTrue);
    client.incoming.add(Uint8List.fromList(reply.sublist(7)));
    expect(await result, isTrue);
    expect(controller.telemetry.position!.x, 1.5);
    expect(controller.telemetry.flightMode, 6);
  });
  test('estimate and info requests decode and merge telemetry', () async {
    final estimate = controller.requestStateEstimate();
    expect(client.sent.last[2], 0x44);
    final data = ByteData(48)
      ..setFloat32(12, 2.5, Endian.little)
      ..setFloat32(24, 0.3, Endian.little)
      ..setFloat32(36, 4, Endian.little);
    client.reply(0x04, data.buffer.asUint8List());
    expect(await estimate, isTrue);
    final info = controller.requestInfo();
    expect(client.sent.last[2], 0x43);
    final details = ByteData(20)
      ..setUint8(0, 0x55)
      ..setUint8(1, 4)
      ..setUint32(4, 20260527, Endian.little)
      ..setFloat32(8, 80, Endian.little);
    client.reply(0x03, details.buffer.asUint8List());
    expect(await info, isTrue);
    expect(controller.telemetry.velocity!.x, 2.5);
    expect(controller.telemetry.angularRate!.x, 4);
    expect(controller.telemetry.batteryPercent, 80);
    expect(controller.telemetry.firmwareVersion, 20260527);
  });
  test('rejection and disconnect complete pending requests', () async {
    final rejected = controller.requestInfo();
    client.reply(0x3D, Uint8List(0));
    expect(await rejected, isFalse);
    final cancelled = controller.requestState();
    await controller.disconnect();
    expect(await cancelled, isFalse);
  });
  test('missing reply times out and releases request slot', () async {
    expect(await controller.requestInfo(), isFalse);
    expect(
      controller.log.any((line) => line.contains('request timed out')),
      isTrue,
    );
    // Automatic polling may immediately start the next INFO request.
  });
}
