import 'dart:async';
import 'package:flutter/material.dart';
import '../services/tcp_client.dart';
import '../widgets/log_view.dart';
import '../widgets/connect_panel.dart';

// We get here through main(). 
// The main GUI screen, its purpose is to show the screen layout for 
// connection between device and ESP-32, including:
//  1. Send 'A' button
//  2. Drone printout log
//  3. IP + Port section
//  4. Connect/Disconnect button
//  5. Landscape/portrait adaptive layout, for ease of use
// 
// Listens to TCPClient and updates log
// Controls when to connect/disconnect
// TLDR: contains the logic that ties UI -> TCP client -> log output.
class DroneTcpConsole extends StatefulWidget {
  const DroneTcpConsole({super.key});
  @override
  State<DroneTcpConsole> createState() => _DroneTcpConsoleState();
}

class _DroneTcpConsoleState extends State<DroneTcpConsole> {
  final ipCtrl = TextEditingController(text: '192.168.0.2'); // put ESP32 IP here
  final portCtrl = TextEditingController(text: '1234');       // match ESP32 server
  final _client = TcpClient();

  final _log = <String>[];
  StreamSubscription<String>? _sub;
  bool _connecting = false;

  void _append(String line) => setState(() => _log.insert(0, line));

  Future<void> _connect() async {
    final ip = ipCtrl.text.trim();
    final port = int.tryParse(portCtrl.text.trim()) ?? 1234;

    if (ip.isEmpty) { _append('Please enter an IP address.'); return; }
    if (_client.isConnected || _connecting) return;

    setState(() => _connecting = true);
    _append('Connecting to $ip:$port…');

    try {
      await _client.connect(ip, port);
      _append('Connected ✅');

      _sub?.cancel();
      _sub = _client.lines.listen(_append);
    } catch (e) {
      _append('Connection failed: $e');
    } finally {
      if (mounted) setState(() => _connecting = false);
    }
  }

  void _disconnect() {
    _client.disconnect();
    _sub?.cancel();
    _sub = null;
    setState(() {});
  }

  void _sendA() {
    if (!_client.isConnected) { _append('Not connected.'); return; }
    _client.sendBytes([0x41]); // 'A'
    _append("Sent 'A'");
  }

  @override
  void dispose() {
    _sub?.cancel();
    _client.dispose();
    ipCtrl.dispose();
    portCtrl.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final connected = _client.isConnected;

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
                    SizedBox(
                      width: double.infinity,
                      height: 56,
                      child: OutlinedButton(
                        onPressed: _sendA,
                        child: const Text("Send 'A'", style: TextStyle(fontSize: 20)),
                      ),
                    ),
                    const SizedBox(height: 20),
                    Text('Drone Printout:', style: Theme.of(context).textTheme.titleMedium),
                    const SizedBox(height: 8),
                    Expanded(child: LogView(lines: _log)),
                  ],
                ),
              ),
            );

            final rightPane = ConnectPanel(
              ipCtrl: ipCtrl,
              portCtrl: portCtrl,
              connected: connected,
              connecting: _connecting,
              onConnect: _connect,
              onDisconnect: _disconnect,
            );

            return Padding(
              padding: const EdgeInsets.all(16),
              child: Flex(
                direction: isLandscape ? Axis.horizontal : Axis.vertical,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Flexible(flex: 3, child: leftPane),
                  if (isLandscape) const SizedBox(width: 16) else const SizedBox(height: 16),
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
