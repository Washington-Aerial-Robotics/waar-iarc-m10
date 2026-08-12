import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../controllers/drone_controller.dart';
import '../controllers/voice_command_controller.dart';
import '../protocol/drone_protocol.dart';

class VoiceControlPanel extends StatelessWidget {
  const VoiceControlPanel({
    super.key,
  });

  @override
  Widget build(BuildContext context) {
    final voice = context.watch<VoiceCommandController>();
    final drone = context.watch<DroneController>();

    // Qualification voice commands (launch/begin orbit/orbit hold/land/
    // abort) don't need the manual joystick TX loop running - the
    // firmware's own onboard state machine drives the flight, not
    // continuous joystick packets - so this panel is usable in
    // Qualification mode without arming the manual control loop first.
    final inQualificationMode =
        drone.telemetry.autonomyMode == DroneComms.autonomyQualification;
    final enabled = drone.isConnected &&
        !voice.initializing &&
        (drone.isControlLoopRunning || inQualificationMode);

    return Card(
      elevation: 2,
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Row(
              children: [
                Icon(
                  voice.listening
                      ? Icons.mic
                      : Icons.mic_none,
                  color: voice.listening
                      ? Colors.red
                      : null,
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: Text(
                    voice.listening
                        ? 'Listening…'
                        : 'Voice Control',
                    style: Theme.of(context)
                        .textTheme
                        .titleMedium,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 8),
            Text(
              inQualificationMode
                  ? 'Say: qualification launch, begin orbit, orbit hold, '
                      'qualification land, or abort qualification.'
                  : drone.isControlLoopRunning
                      ? 'Say: up, down, hover, stop, forward, '
                          'backward, move left, move right, '
                          'rotate left, or rotate right.'
                      : 'ARM the drone before using voice commands.',
              style: const TextStyle(
                fontSize: 13,
              ),
            ),
            const SizedBox(height: 10),
            if (voice.recognizedText.isNotEmpty)
              Container(
                width: double.infinity,
                padding: const EdgeInsets.all(8),
                decoration: BoxDecoration(
                  color: Colors.black.withOpacity(0.04),
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(
                    color: Colors.black12,
                  ),
                ),
                child: Text(
                  'Heard: ${voice.recognizedText}',
                  style: const TextStyle(
                    fontFamily: 'monospace',
                  ),
                ),
              ),
            if (voice.lastCommand.isNotEmpty) ...[
              const SizedBox(height: 8),
              Text(
                'Command: ${voice.lastCommand}',
                style: const TextStyle(
                  fontWeight: FontWeight.bold,
                ),
              ),
            ],
            if (voice.error != null) ...[
              const SizedBox(height: 8),
              Text(
                voice.error!,
                style: const TextStyle(
                  color: Colors.red,
                ),
              ),
            ],
            const SizedBox(height: 10),
            ElevatedButton.icon(
              onPressed: !enabled
                  ? null
                  : voice.listening
                      ? voice.stopListening
                      : voice.startListening,
              icon: Icon(
                voice.listening
                    ? Icons.stop
                    : Icons.mic,
              ),
              label: Text(
                voice.initializing
                    ? 'Initializing…'
                    : voice.listening
                        ? 'Stop listening'
                        : 'Speak command',
              ),
            ),
          ],
        ),
      ),
    );
  }
}