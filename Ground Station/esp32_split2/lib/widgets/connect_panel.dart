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
    void finishEditing() => FocusManager.instance.primaryFocus?.unfocus();

    Widget confirmButton() => IconButton(
      tooltip: 'Done editing',
      icon: const Icon(Icons.check),
      onPressed: finishEditing,
    );
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
              textInputAction: TextInputAction.done,
              autocorrect: false,
              enableSuggestions: false,
              onSubmitted: (_) => finishEditing(),
              onTapOutside: (_) => finishEditing(),
              maxLength: 1,
              textCapitalization: TextCapitalization.characters,
              decoration: InputDecoration(
                hintText: 'A',
                suffixIcon: confirmButton(),
                helperText: 'Single-character drone ID',
                border: const OutlineInputBorder(),
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
              textInputAction: TextInputAction.done,
              autocorrect: false,
              enableSuggestions: false,
              onSubmitted: (_) => finishEditing(),
              onTapOutside: (_) => finishEditing(),
              decoration: InputDecoration(
                hintText: 'Kent default: 192.168.1.240',
                suffixIcon: confirmButton(),
                border: const OutlineInputBorder(),
                isDense: true,
              ),
              enabled: !connected && !connecting,
            ),
            const SizedBox(height: 12),
            Text('Port:', style: Theme.of(context).textTheme.titleMedium),
            const SizedBox(height: 8),
            TextField(
              controller: portCtrl,
              textInputAction: TextInputAction.done,
              autocorrect: false,
              enableSuggestions: false,
              onSubmitted: (_) => finishEditing(),
              onTapOutside: (_) => finishEditing(),
              keyboardType: TextInputType.number,
              decoration: InputDecoration(
                hintText: 'Kent TCP port: 70',
                suffixIcon: confirmButton(),
                border: const OutlineInputBorder(),
                isDense: true,
              ),
              enabled: !connected && !connecting,
            ),
            const SizedBox(height: 16),
            SizedBox(
              height: 80,
              child: ElevatedButton(
                onPressed: connected
                    ? () {
                        finishEditing();
                        onDisconnect();
                      }
                    : (connecting
                          ? null
                          : () {
                              finishEditing();
                              onConnect();
                            }),
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
