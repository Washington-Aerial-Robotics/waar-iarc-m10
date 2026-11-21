import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../controllers/drone_controller.dart';

/// Reported mission state only; receipt replies are not acceptance.
class MissionPanel extends StatelessWidget {
  const MissionPanel({super.key});

  @override
  Widget build(BuildContext context) {
    final controller = context.watch<DroneController>();
    final telemetry = controller.telemetry;
    final fresh = controller.missionInfoFresh;
    final configure =
        fresh && telemetry.armed == false && !controller.missionCommandPending;
    final normal = fresh && !controller.missionCommandPending;
    const qualStates = [
      'BOOT',
      'CLIMB',
      'HOVER',
      'ORBIT',
      'POST_HOLD',
      'LANDING',
      'FINISH',
    ];
    const squareStates = [
      'BOOT',
      'CLIMB',
      'LEG1',
      'LEG2',
      'LEG3',
      'LEG4',
      'LANDING',
      'FINISH',
    ];
    String stateName(List<String> names, int? state) =>
        state != null && state >= 0 && state < names.length
        ? names[state]
        : 'Unknown';
    Widget action(
      String label,
      VoidCallback onPressed, {
      bool enabled = true,
      bool danger = false,
    }) => ElevatedButton(
      onPressed: enabled ? onPressed : null,
      style: danger
          ? ElevatedButton.styleFrom(
              backgroundColor: Colors.red,
              foregroundColor: Colors.white,
            )
          : null,
      child: Text(label),
    );
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          Text(
            fresh
                ? 'Live INFO · firmware v0.9.0+ (20-byte payload)'
                : 'No fresh INFO — connect in Console; mission state unknown',
          ),
          const SizedBox(height: 12),
          Wrap(
            spacing: 8,
            runSpacing: 4,
            children: [
              for (var i = 0; i < 4; i++)
                Chip(
                  avatar: Icon(
                    fresh && ((telemetry.sensorStatus ?? 0) & (1 << i)) != 0
                        ? Icons.check_circle
                        : Icons.help_outline,
                    color:
                        fresh && ((telemetry.sensorStatus ?? 0) & (1 << i)) != 0
                        ? Colors.green
                        : Colors.orange,
                  ),
                  label: Text(['GPS', 'BARO', 'IMU', 'MAG'][i]),
                ),
              Chip(
                label: Text(
                  !fresh || telemetry.armed == null
                      ? 'Armed: unknown'
                      : telemetry.armed!
                      ? 'ARMED'
                      : 'DISARMED',
                ),
              ),
              Chip(
                label: Text(
                  fresh && telemetry.batteryPercent != null
                      ? 'Battery ${telemetry.batteryPercent!.toStringAsFixed(1)}%'
                      : 'Battery unknown',
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),
          DropdownButtonFormField<int>(
            initialValue:
                telemetry.autonomyMode != null &&
                    telemetry.autonomyMode! >= 0 &&
                    telemetry.autonomyMode! <= 3
                ? telemetry.autonomyMode
                : null,
            key: ValueKey('mode-${telemetry.autonomyMode}'),
            decoration: const InputDecoration(
              labelText: 'Reported autonomy mode',
            ),
            items: [
              for (var i = 0; i < 4; i++)
                DropdownMenuItem(
                  value: i,
                  child: Text(['MANUAL', 'QUAL', 'MINE SEARCH', 'SQUARE'][i]),
                ),
            ],
            onChanged: configure
                ? (value) {
                    if (value != null) controller.setAutonomyMode(value);
                  }
                : null,
          ),
          const SizedBox(height: 12),
          DropdownButtonFormField<int>(
            key: ValueKey('slot-${telemetry.formationSlot}'),
            initialValue:
                telemetry.formationSlot != null &&
                    telemetry.formationSlot! >= 0 &&
                    telemetry.formationSlot! <= 3
                ? telemetry.formationSlot
                : null,
            decoration: const InputDecoration(
              labelText: 'Reported formation slot',
            ),
            items: [
              for (var i = 0; i < 4; i++)
                DropdownMenuItem(value: i, child: Text('Slot $i')),
            ],
            onChanged: configure
                ? (value) {
                    if (value != null) controller.setFormationSlot(value);
                  }
                : null,
          ),
          const SizedBox(height: 16),
          Text(
            'QUAL: ${stateName(qualStates, telemetry.qualState)} · Revolutions: ${telemetry.qualRevolutions ?? "?"}',
          ),
          Wrap(
            spacing: 8,
            children: [
              action(
                'LAUNCH',
                () {
                  controller.sendQualCommand(1);
                },
                enabled:
                    normal &&
                    telemetry.autonomyMode == 1 &&
                    controller.sensorsReady,
              ),
              action('BEGIN ORBIT', () {
                controller.sendQualCommand(2);
              }, enabled: normal && telemetry.autonomyMode == 1),
              action('HOLD', () {
                controller.sendQualCommand(3);
              }, enabled: normal && telemetry.autonomyMode == 1),
              action('QUAL LAND', () {
                controller.sendQualCommand(4);
              }, enabled: controller.isConnected),
              action(
                'QUAL ABORT',
                () {
                  controller.sendQualCommand(5);
                },
                enabled: controller.isConnected,
                danger: true,
              ),
            ],
          ),
          const SizedBox(height: 16),
          Text('SQUARE: ${stateName(squareStates, telemetry.squareState)}'),
          Wrap(
            spacing: 8,
            children: [
              action(
                'START',
                () {
                  controller.sendSquareCommand(1);
                },
                enabled:
                    normal &&
                    telemetry.autonomyMode == 3 &&
                    controller.sensorsReady,
              ),
              action('SQUARE LAND', () {
                controller.sendSquareCommand(2);
              }, enabled: controller.isConnected),
              action(
                'SQUARE ABORT',
                () {
                  controller.sendSquareCommand(3);
                },
                enabled: controller.isConnected,
                danger: true,
              ),
              action(
                'KILL',
                controller.kill,
                enabled: controller.isConnected,
                danger: true,
              ),
            ],
          ),
          const SizedBox(height: 16),
          Text(controller.missionStatus),
          const SizedBox(height: 8),
          const Text(
            'Receipt is not acceptance. Confirm using reported INFO. LAND/ABORT are always sent when connected, but firmware still requires the corresponding mission mode. KILL restarts the ESP32. Test with propellers removed.',
          ),
        ],
      ),
    );
  }
}
