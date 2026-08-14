import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../controllers/drone_controller.dart';
import '../protocol/drone_protocol.dart';
import '../widgets/voice_control_panel.dart';

/// MANUAL/QUALIFICATION/MINE_SEARCH mode selection, formation slot
/// assignment, and the high-level qualification commands (LAUNCH/BEGIN
/// ORBIT/HOLD/LAND/ABORT). Deliberately does no trajectory/flight-control
/// computation itself - it only sends the mode/slot/command bytes the
/// firmware's onboard qualification state machine (commander.cpp) acts on,
/// and displays whatever that state machine reports back over telemetry.
class QualificationControl extends StatelessWidget {
  const QualificationControl({super.key});

  static const _modes = [
    (DroneComms.autonomyManual, 'Manual', Icons.sports_esports),
    (DroneComms.autonomyQualification, 'Qualification', Icons.flight_takeoff),
    (DroneComms.autonomyMineSearch, 'Mine Search', Icons.travel_explore),
    (DroneComms.autonomySquareTest, 'Square Test', Icons.crop_square),
  ];

  static const _qualStateLabels = {
    DroneComms.qualStateBoot: 'BOOT - waiting for launch',
    DroneComms.qualStateClimbToFormation: 'CLIMB TO FORMATION',
    DroneComms.qualStateHoverHold: 'HOVER HOLD - waiting for orbit',
    DroneComms.qualStateOrbit: 'ORBIT',
    DroneComms.qualStatePostOrbitHold: 'POST-ORBIT HOLD - waiting for land',
    DroneComms.qualStateLanding: 'LANDING',
    DroneComms.qualStateFinish: 'FINISHED - landed',
  };

  static const _squareStateLabels = {
    DroneComms.squareStateBoot: 'BOOT - waiting for start',
    DroneComms.squareStateClimb: 'CLIMB',
    DroneComms.squareStateLeg1: 'LEG 1',
    DroneComms.squareStateLeg2: 'LEG 2',
    DroneComms.squareStateLeg3: 'LEG 3',
    DroneComms.squareStateLeg4: 'LEG 4 - returning to origin',
    DroneComms.squareStateLanding: 'LANDING',
    DroneComms.squareStateFinish: 'FINISHED - landed',
  };

  @override
  Widget build(BuildContext context) {
    final ctrl = context.watch<DroneController>();
    final telemetry = ctrl.telemetry;
    final autonomyMode = telemetry.autonomyMode ?? DroneComms.autonomyManual;
    final armed = telemetry.armed ?? false;

    return Scaffold(
      appBar: AppBar(title: const Text('Mode')),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Text('Operating Mode', style: Theme.of(context).textTheme.titleMedium),
            const SizedBox(height: 8),
            SegmentedButton<int>(
              segments: [
                for (final (value, label, icon) in _modes)
                  ButtonSegment(value: value, label: Text(label), icon: Icon(icon)),
              ],
              selected: {autonomyMode},
              // Mode selection alone never arms/launches anything - it just
              // tells the firmware which behavior is selected. Disabled
              // while armed so a mode can't be swapped mid-flight (firmware
              // rejects this anyway, but disabling avoids a confusing no-op
              // tap).
              onSelectionChanged: armed
                  ? null
                  : (selection) => ctrl.setAutonomyMode(selection.first),
            ),
            if (armed)
              const Padding(
                padding: EdgeInsets.only(top: 4),
                child: Text(
                  'Disarm to change mode',
                  style: TextStyle(color: Colors.orange),
                ),
              ),
            const Divider(height: 32),
            if (autonomyMode == DroneComms.autonomyQualification) ...[
              _QualificationPanel(ctrl: ctrl, armed: armed),
            ] else if (autonomyMode == DroneComms.autonomySquareTest) ...[
              _SquareTestPanel(ctrl: ctrl, armed: armed),
            ] else if (autonomyMode == DroneComms.autonomyMineSearch) ...[
              const Text('Mine Search mode selected. Onboard survey/mapping '
                  'behavior is not implemented yet - this mode currently '
                  'does nothing on the drone.'),
            ] else ...[
              const Text('Manual mode selected - use the Remote tab for '
                  'joystick control.'),
            ],
          ],
        ),
      ),
    );
  }
}

class _QualificationPanel extends StatelessWidget {
  const _QualificationPanel({required this.ctrl, required this.armed});

  final DroneController ctrl;
  final bool armed;

  @override
  Widget build(BuildContext context) {
    final telemetry = ctrl.telemetry;
    final qualState = telemetry.qualState ?? DroneComms.qualStateBoot;
    final revolutions = telemetry.qualRevolutions ?? 0;
    final formationSlot = telemetry.formationSlot ?? 0;
    final stateLabel = QualificationControl._qualStateLabels[qualState] ??
        'Unknown ($qualState)';

    final canLaunch = qualState == DroneComms.qualStateBoot;
    final canBeginOrbit = qualState == DroneComms.qualStateHoverHold;
    final canHold = qualState == DroneComms.qualStateOrbit;
    final canLandOrAbort = qualState != DroneComms.qualStateLanding &&
        qualState != DroneComms.qualStateFinish;

    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Text('Formation Slot', style: Theme.of(context).textTheme.titleMedium),
        const SizedBox(height: 8),
        SegmentedButton<int>(
          segments: const [
            ButtonSegment(value: 0, label: Text('0')),
            ButtonSegment(value: 1, label: Text('1')),
            ButtonSegment(value: 2, label: Text('2')),
            ButtonSegment(value: 3, label: Text('3')),
          ],
          selected: {formationSlot},
          onSelectionChanged:
              armed ? null : (s) => ctrl.setFormationSlot(s.first),
        ),
        const SizedBox(height: 24),
        Card(
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('Status', style: Theme.of(context).textTheme.titleMedium),
                const SizedBox(height: 8),
                Text('State: $stateLabel'),
                Text('Formation slot: $formationSlot'),
                Text('Revolutions: $revolutions / 10'),
                if (telemetry.position != null)
                  Text(
                    'Altitude: ${telemetry.position!.z.toStringAsFixed(2)} m',
                  ),
              ],
            ),
          ),
        ),
        const SizedBox(height: 24),
        const VoiceControlPanel(),
        const SizedBox(height: 24),
        Text('Commands', style: Theme.of(context).textTheme.titleMedium),
        const SizedBox(height: 8),
        FilledButton.icon(
          icon: const Icon(Icons.flight_takeoff),
          label: const Text('LAUNCH'),
          onPressed: canLaunch
              ? () => ctrl.sendQualCommand(DroneComms.qualCmdLaunch)
              : null,
        ),
        const SizedBox(height: 8),
        FilledButton.icon(
          icon: const Icon(Icons.sync),
          label: const Text('BEGIN ORBIT'),
          onPressed: canBeginOrbit
              ? () => ctrl.sendQualCommand(DroneComms.qualCmdBeginOrbit)
              : null,
        ),
        const SizedBox(height: 8),
        FilledButton.icon(
          icon: const Icon(Icons.pause_circle),
          label: const Text('HOLD'),
          onPressed: canHold
              ? () => ctrl.sendQualCommand(DroneComms.qualCmdHold)
              : null,
        ),
        const SizedBox(height: 16),
        // LAND/ABORT have priority over every other qualification command
        // on the firmware side - kept visually distinct here for the same
        // reason, and enabled from any in-progress state.
        FilledButton.icon(
          style: FilledButton.styleFrom(backgroundColor: Colors.orange),
          icon: const Icon(Icons.flight_land),
          label: const Text('LAND'),
          onPressed: canLandOrAbort
              ? () => ctrl.sendQualCommand(DroneComms.qualCmdLand)
              : null,
        ),
        const SizedBox(height: 8),
        FilledButton.icon(
          style: FilledButton.styleFrom(backgroundColor: Colors.red),
          icon: const Icon(Icons.warning),
          label: const Text('ABORT'),
          onPressed: canLandOrAbort
              ? () => ctrl.sendQualCommand(DroneComms.qualCmdAbort)
              : null,
        ),
      ],
    );
  }
}

/// GPS+BME position-hold validation maneuver (fly a 4-corner square and
/// return to the launch point) - a simpler pattern than the rules-mandated
/// circular orbit, meant for exercising GPS x/y + BME z position control
/// before ever attempting Qualification's real orbit. Not part of the
/// competition itself.
class _SquareTestPanel extends StatelessWidget {
  const _SquareTestPanel({required this.ctrl, required this.armed});

  final DroneController ctrl;
  final bool armed;

  @override
  Widget build(BuildContext context) {
    final telemetry = ctrl.telemetry;
    final squareState = telemetry.squareState ?? DroneComms.squareStateBoot;
    final stateLabel = QualificationControl._squareStateLabels[squareState] ??
        'Unknown ($squareState)';

    final canStart = squareState == DroneComms.squareStateBoot;
    final canLandOrAbort = squareState != DroneComms.squareStateLanding &&
        squareState != DroneComms.squareStateFinish;

    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Card(
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('Status', style: Theme.of(context).textTheme.titleMedium),
                const SizedBox(height: 8),
                Text('State: $stateLabel'),
                if (telemetry.position != null) ...[
                  Text(
                    'Position: (${telemetry.position!.x.toStringAsFixed(2)}, '
                    '${telemetry.position!.y.toStringAsFixed(2)}) m',
                  ),
                  Text(
                    'Altitude: ${telemetry.position!.z.toStringAsFixed(2)} m',
                  ),
                ],
              ],
            ),
          ),
        ),
        const SizedBox(height: 24),
        Text('Commands', style: Theme.of(context).textTheme.titleMedium),
        const SizedBox(height: 8),
        FilledButton.icon(
          icon: const Icon(Icons.flight_takeoff),
          label: const Text('START'),
          onPressed: canStart
              ? () => ctrl.sendSquareCommand(DroneComms.squareCmdStart)
              : null,
        ),
        const SizedBox(height: 16),
        // LAND/ABORT have priority over START on the firmware side, same
        // reasoning as the Qualification panel above.
        FilledButton.icon(
          style: FilledButton.styleFrom(backgroundColor: Colors.orange),
          icon: const Icon(Icons.flight_land),
          label: const Text('LAND'),
          onPressed: canLandOrAbort
              ? () => ctrl.sendSquareCommand(DroneComms.squareCmdLand)
              : null,
        ),
        const SizedBox(height: 8),
        FilledButton.icon(
          style: FilledButton.styleFrom(backgroundColor: Colors.red),
          icon: const Icon(Icons.warning),
          label: const Text('ABORT'),
          onPressed: canLandOrAbort
              ? () => ctrl.sendSquareCommand(DroneComms.squareCmdAbort)
              : null,
        ),
      ],
    );
  }
}
