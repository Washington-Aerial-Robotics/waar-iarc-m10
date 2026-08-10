import 'dart:async';
import 'package:flutter/material.dart';
import '../services/tcp_client.dart';
import '../widgets/log_view.dart';
import '../widgets/connect_panel.dart';
import 'package:provider/provider.dart';
import '../protocol/drone_protocol.dart';
import '../controllers/drone_controller.dart';

class DroneTcpConsole extends StatefulWidget {
  const DroneTcpConsole({super.key});

  @override
  State<DroneTcpConsole> createState() => _DroneTcpConsoleState();
}

class _DroneTcpConsoleState extends State<DroneTcpConsole> {
  final ipCtrl = TextEditingController(text: '172.20.10.8');
  final portCtrl = TextEditingController(text: '70');

  late TcpClient _client;
  late final DronePacketBuilder _packetBuilder;

  static const int _fromId = 0x47; // 'G'
  // static const int _toId = 0x42;   // 'B'
  final droneIdCtrl = TextEditingController(text: 'U');

  final _log = <String>[];
  StreamSubscription<String>? _sub;
  bool _connecting = false;

  @override
  void initState() {
    super.initState();

    _client = context.read<TcpClient>();
    final ctrl = context.read<DroneController>();
    droneIdCtrl.text = ctrl.targetDroneCharacter;

    _packetBuilder = DronePacketBuilder(
      fromId: DroneController.appDeviceId,
      defaultToId: ctrl.targetDroneId,
    );

    if (_client.isConnected) {
      _sub = _client.lines.listen(_append);
    }
  }

  void _append(String line) {
    if (!mounted) return;
    setState(() => _log.insert(0, line));
  }

  Future<void> _connect() async {
    final ip = ipCtrl.text.trim();
    final port = int.tryParse(portCtrl.text.trim()) ?? 70;
    final ctrl = context.read<DroneController>();

    setState(() => _connecting = true);

    try {
      await ctrl.connect(ip, port);

      await _sub?.cancel();
      _sub = _client.lines.listen(_append);

      _append('Controller connected');
    } catch (e) {
      _append('Connect failed: $e');
    } finally {
      if (mounted) {
        setState(() => _connecting = false);
      }
    }
  }

  Future<void> _disconnect() async {
    final ctrl = context.read<DroneController>();

    await ctrl.disconnect();

    await _sub?.cancel();
    _sub = null;

    _append('Controller disconnected');
  }

  void _changeDroneId(String input) {
    if (input.trim().isEmpty) {
      return;
    }

    final ctrl = context.read<DroneController>();
    final accepted = ctrl.setTargetDroneId(input);

    if (accepted) {
      final normalized = ctrl.targetDroneCharacter;

      if (droneIdCtrl.text != normalized) {
        droneIdCtrl.value = TextEditingValue(
          text: normalized,
          selection: TextSelection.collapsed(
            offset: normalized.length,
          ),
        );
      }

      _append('Target drone set to $normalized');
    }
  }

  void _sendPing() {
    if (!_client.isConnected) {
      _append('Not connected.');
      return;
    }

    final ctrl = context.read<DroneController>();

    final pkt = _packetBuilder.headerOnly(
      messageType: DroneComms.comPing,
      toId: ctrl.targetDroneId
    );

    _client.sendBytes(pkt);
    _append(_formatPacket(
      'Sent COM_PING to ${ctrl.targetDroneCharacter}',
      pkt,
    ));
  }

  void _requestWifiIp() {
    if (!_client.isConnected) {
      _append('Not connected.');
      return;
    }

    final ctrl = context.read<DroneController>();

    final pkt = _packetBuilder.headerOnly(
      messageType: DroneComms.comRequestWifi,
      toId: ctrl.targetDroneId,
    );

    _client.sendBytes(pkt);
    _append(_formatPacket(
      'Sent COM_REQUEST_WIFI to ${ctrl.targetDroneCharacter}',
      pkt,
    ));
  }

  void _requestPosition() {
    if (!_client.isConnected) {
      _append('Not connected.');
      return;
    }

    final ctrl = context.read<DroneController>();

    final pkt = _packetBuilder.headerOnly(
      messageType: DroneComms.comRequestPos,
      toId: ctrl.targetDroneId,
    );

    _client.sendBytes(pkt);
    _append(_formatPacket(
      'Sent COM_REQUEST_POS to ${ctrl.targetDroneCharacter}',
      pkt,
    ));
  }

  void _requestInfo() {
    if (!_client.isConnected) {
      _append('Not connected.');
      return;
    }

    final ctrl = context.read<DroneController>();

    final pkt = _packetBuilder.headerOnly(
      messageType: DroneComms.comRequestInfo,
      toId: ctrl.targetDroneId,
    );

    _client.sendBytes(pkt);
    _append(_formatPacket(
      'Sent COM_REQUEST_INFO to ${ctrl.targetDroneCharacter}',
      pkt,
    ));
  }

  void _sendTestControlBytes() {
    if (!_client.isConnected) {
      _append('Not connected.');
      return;
    }

    final ctrl = context.read<DroneController>();

    final pkt = _packetBuilder.motor4Floats(
      m0: 0.5,
      m1: 0.5,
      m2: 0.5,
      m3: 0.5,
      toId: ctrl.targetDroneId,
    );

    _client.sendBytes(pkt);
    _append(_formatPacket(
      'Sent TEST CONTROL FLOATS to ${ctrl.targetDroneCharacter}',
      pkt,
    ));
  }

  String _formatPacket(String label, List<int> pkt) {
    final hex = pkt
        .map((b) => b.toRadixString(16).padLeft(2, '0').toUpperCase())
        .join(' ');

    return '$label: $hex';
  }

  @override
  void dispose() {
    _sub?.cancel();
    ipCtrl.dispose();
    portCtrl.dispose();
    droneIdCtrl.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final connected = _client.isConnected;
    final droneCtrl = context.watch<DroneController>();

    return Scaffold(
      appBar: AppBar(
        title: const Text('Drone Wi-Fi Console'),
        centerTitle: true,
      ),
      body: SafeArea(
        child: OrientationBuilder(
          builder: (context, orientation) {
            final isLandscape = orientation == Orientation.landscape;

            final leftPane = Card(
              elevation: 2,
              child: Padding(
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      children: [
                        Expanded(
                          child: SizedBox(
                            height: 56,
                            child: OutlinedButton(
                              onPressed: _sendPing,
                              child: const Text(
                                'Send PING',
                                style: TextStyle(fontSize: 18),
                              ),
                            ),
                          ),
                        ),
                        const SizedBox(width: 12),
                        Expanded(
                          child: SizedBox(
                            height: 56,
                            child: OutlinedButton(
                              onPressed: _requestWifiIp,
                              child: const Text(
                                'Get IP',
                                style: TextStyle(fontSize: 18),
                              ),
                            ),
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 12),
                    Row(
                      children: [
                        Expanded(
                          child: SizedBox(
                            height: 56,
                            child: OutlinedButton(
                              onPressed: _requestPosition,
                              child: const Text(
                                'Get Pos',
                                style: TextStyle(fontSize: 18),
                              ),
                            ),
                          ),
                        ),
                        const SizedBox(width: 12),
                        Expanded(
                          child: SizedBox(
                            height: 56,
                            child: OutlinedButton(
                              onPressed: _sendTestControlBytes,
                              child: const Text(
                                'Test Control',
                                style: TextStyle(fontSize: 18),
                              ),
                            ),
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 12),
                    Row(
                      children: [
                        Expanded(
                          child: SizedBox(
                            height: 56,
                            child: OutlinedButton(
                              onPressed: _requestInfo,
                              child: const Text(
                                'Get Info',
                                style: TextStyle(fontSize: 18),
                              ),
                            ),
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 16),
                    Text(
                      'Telemetry:',
                      style: Theme.of(context).textTheme.titleMedium,
                    ),
                    const SizedBox(height: 8),
                    Container(
                      width: double.infinity,
                      padding: const EdgeInsets.all(10),
                      decoration: BoxDecoration(
                        color: Colors.black.withOpacity(0.04),
                        borderRadius: BorderRadius.circular(8),
                        border: Border.all(color: Colors.black12),
                      ),
                      child: Text(
                        '${droneCtrl.telemetry}',
                        style: const TextStyle(
                          fontFamily: 'monospace',
                          fontSize: 13,
                        ),
                      ),
                    ),
                    const SizedBox(height: 20),
                    Text(
                      'Drone Printout:',
                      style: Theme.of(context).textTheme.titleMedium,
                    ),
                    const SizedBox(height: 8),
                    Expanded(child: LogView(lines: _log)),
                  ],
                ),
              ),
            );

            final rightPane = SingleChildScrollView(
               child: ConnectPanel(
                ipCtrl: ipCtrl,
                portCtrl: portCtrl,
                droneIdCtrl: droneIdCtrl,
                connected: connected,
                connecting: _connecting,
                onConnect: _connect,
                onDisconnect: _disconnect,
                onDroneIdChanged: _changeDroneId,
              ),
            );

            return Padding(
              padding: const EdgeInsets.all(16),
              child: Flex(
                direction: isLandscape ? Axis.horizontal : Axis.vertical,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Flexible(flex: 3, child: leftPane),
                  if (isLandscape)
                    const SizedBox(width: 16)
                  else
                    const SizedBox(height: 16),
                  Flexible(flex: 2, child: rightPane),
                ],
              ),
            );
          },
        ),
      ),
    );
  }
}
