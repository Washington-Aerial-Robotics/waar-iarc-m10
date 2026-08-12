import 'package:flutter/material.dart';

/// Reusable connection settings panel.
///
/// Responsibilities:
/// - Shows input fields for Drone IP address and TCP port.
/// - Displays a Connect/Disconnect button with proper enabled/disabled states.
/// - Exposes callbacks for connect/disconnect actions.
///
/// This widget contains no socket logic; it is purely a UI component.

class ConnectPanel extends StatelessWidget {
  const ConnectPanel({
  super.key,
  required this.ipCtrl,
  required this.portCtrl,
  required this.droneIdCtrl,
  required this.connected,
  required this.connecting,
  required this.onConnect,
  required this.onDisconnect,
  required this.onDroneIdChanged,
});

  final TextEditingController ipCtrl;
  final TextEditingController portCtrl;
  final TextEditingController droneIdCtrl;

  final bool connected;
  final bool connecting;

  final VoidCallback onConnect;
  final VoidCallback onDisconnect;
  final ValueChanged<String> onDroneIdChanged;
  

  @override
  Widget build(BuildContext context) {
    // Find a way to make this widget scrollable, want the user to be able to reach the connect to drone button
    return Card(
      elevation: 2,
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Text(
                  'Target Drone ID:',
                  style: Theme.of(context).textTheme.titleMedium,
                ),
                const SizedBox(height: 8),
                TextField(
                  controller: droneIdCtrl,
                  maxLength: 1,
                  textCapitalization: TextCapitalization.characters,
                  decoration: const InputDecoration(
                    hintText: 'A',
                    helperText: 'Single-character drone ID',
                    border: OutlineInputBorder(),
                    isDense: true,
                    counterText: '',
                  ),
                  onChanged: onDroneIdChanged,
                ),
                const SizedBox(height: 12),
            Text('Drone IP:', style: Theme.of(context).textTheme.titleMedium),
            const SizedBox(height: 8),
            TextField(
              controller: ipCtrl,
              decoration: const InputDecoration(
                hintText: 'e.g., 172.20.10.7 or 192.168.4.1',
                border: OutlineInputBorder(),
                isDense: true,
              ),
              enabled: !connected && !connecting,
            ),
            const SizedBox(height: 12),
            Text('Port:', style: Theme.of(context).textTheme.titleMedium),
            const SizedBox(height: 8),
            TextField(
              controller: portCtrl,
              keyboardType: TextInputType.number,
              decoration: const InputDecoration(
                hintText: '70 (the firmware\'s WiFiServer port)',
                border: OutlineInputBorder(),
                isDense: true,
              ),
              enabled: !connected && !connecting,
            ),
            const SizedBox(height: 16),
            SizedBox(
              height: 80,
              child: ElevatedButton(
                onPressed: connected
                    ? onDisconnect
                    : (connecting ? null : onConnect),
                child: Text(
                  connected
                      ? 'Disconnect'
                      : (connecting ? 'Connecting…' : 'Connect to drone'),
                  style: const TextStyle(fontSize: 18),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
