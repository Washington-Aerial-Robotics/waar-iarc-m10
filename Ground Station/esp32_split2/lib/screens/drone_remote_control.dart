import 'dart:math';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../controllers/drone_controller.dart';
import '../widgets/voice_control_panel.dart';

/// Remote Control screen (primary flight UI).
///
/// Responsibilities:
/// - Displays a dual-joystick controller layout:
///   - Left stick: Throttle / Yaw
///   - Right stick: Pitch / Roll
/// - Shows telemetry readouts and control state to the user.
/// - Provides safety/command buttons (ARM, DISARM, KILL) in the center HUD.
/// - Adapts layout for portrait vs landscape and avoids overlap with the
///   system safe areas and any parent bottom navigation bar.
/// - Sends user input to the `DroneController`, which then communicates with
///   the ESP32.
///
/// This is the main “pilot interface” for controlling the drone.

class DroneRemoteControl extends StatelessWidget {
  const DroneRemoteControl({super.key});

  @override
  Widget build(BuildContext context) {
    final ctrl = context.watch<DroneController>();
    final connected = ctrl.isConnected;
    final isLandscape =
        MediaQuery.of(context).orientation == Orientation.landscape;

    String telemetryText() =>
        'thr=${ctrl.throttle.toStringAsFixed(2)}   '
        'yaw=${ctrl.yaw.toStringAsFixed(2)}   '
        'pit=${ctrl.pitch.toStringAsFixed(2)}   '
        'rol=${ctrl.roll.toStringAsFixed(2)}';

    return Scaffold(
      appBar: AppBar(
        title: const Text('Remote Control'),
        centerTitle: true,
      ),
      body: SafeArea(
        child: LayoutBuilder(
          builder: (context, c) {
            const pad = 8.0;

            if (isLandscape) {
              // -------- LANDSCAPE: Top telemetry + Bottom sticks + center HUD --------
              // Give the top panel a fixed-ish chunk so it never gets covered.
              final topH = (c.maxHeight * 0.28).clamp(86.0, 130.0);
              final bottomH = max(0.0, c.maxHeight - topH - pad);

              // Middle HUD width
              final hudW = (c.maxWidth * 0.22).clamp(210.0, 300.0);

              // Compute stick size from remaining width/height
              final availableW = max(0.0, c.maxWidth - hudW - pad * 2 - 16);
              final eachStickW = availableW / 2;
              // Subtract label space so no overflow
              final stickSize = min(eachStickW, bottomH - 34).clamp(160.0, 460.0);

              return Padding(
                padding: const EdgeInsets.all(pad),
                child: Column(
                  children: [
                    // Top telemetry/status strip (never covered)
                    _TelemetryStrip(
                      connected: connected,
                      text: telemetryText(),
                    ),
                    const SizedBox(height: 8),

                    // Bottom controller row fills remaining space
                    Expanded(
                      child: Row(
                        crossAxisAlignment: CrossAxisAlignment.end,
                        children: [
                          // Left stick flush left
                          SizedBox(
                            width: stickSize,
                            child: _Stick(
                              size: stickSize,
                              label: 'Throttle / Yaw',
                              enabled: connected,
                              onChanged: (x, y) {
                                final thr = ((y + 1) / 2).clamp(0.0, 1.0);
                                ctrl.setSticks(t: thr, y: x);
                              },
                              onRelease: () => ctrl.setSticks(t: 0.0, y: 0.0),
                            ),
                          ),

                          const SizedBox(width: 8),

                          // Center HUD panel between sticks (scrollable if needed)
                          Expanded(
                            child: Center(
                              child: _HudPanel(
                                width: hudW,
                                connected: connected,
                                onArm: connected ? ctrl.arm : null,
                                onDisarm: connected ? ctrl.disarm : null,
                                onKill: connected ? ctrl.kill : null,
                                telemetryMultiline: [
                                  'thr=${ctrl.throttle.toStringAsFixed(2)}',
                                  'yaw=${ctrl.yaw.toStringAsFixed(2)}',
                                  'pit=${ctrl.pitch.toStringAsFixed(2)}',
                                  'rol=${ctrl.roll.toStringAsFixed(2)}',
                                ],
                              ),
                            ),
                          ),

                          const SizedBox(width: 8),
                          const SizedBox(height: 8),
                          const VoiceControlPanel(),

                          // Right stick flush right
                          SizedBox(
                            width: stickSize,
                            child: _Stick(
                              size: stickSize,
                              label: 'Pitch / Roll',
                              enabled: connected,
                              onChanged: (x, y) => ctrl.setSticks(r: x, p: y),
                              onRelease: () => ctrl.setSticks(r: 0.0, p: 0.0),
                            ),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              );
            }

            // PORTRAIT: top panel + sticks row
            final stickSize = min((c.maxWidth - pad * 2 - 12) / 2, c.maxHeight - 170)
                .clamp(180.0, 420.0);

            return Padding(
              padding: const EdgeInsets.all(pad),
              child: Column(
                children: [
                  _PortraitTopPanel(
                    connected: connected,
                    onArm: connected ? ctrl.arm : null,
                    onDisarm: connected ? ctrl.disarm : null,
                    onKill: connected ? ctrl.kill : null,
                    telemetry: telemetryText(),
                  ),
                  const SizedBox(height: 8),
                  const VoiceControlPanel(),
                  const SizedBox(height: 10),
                  Expanded(
                    child: Align(
                      alignment: Alignment.bottomCenter,
                      child: Row(
                        crossAxisAlignment: CrossAxisAlignment.end,
                        mainAxisAlignment: MainAxisAlignment.spaceBetween,
                        children: [
                          SizedBox(
                            width: stickSize,
                            child: _Stick(
                              size: stickSize,
                              label: 'Throttle / Yaw',
                              enabled: connected,
                              onChanged: (x, y) {
                                final thr = ((y + 1) / 2).clamp(0.0, 1.0);
                                ctrl.setSticks(t: thr, y: x);
                              },
                              onRelease: () => ctrl.setSticks(t: 0.0, y: 0.0),
                            ),
                          ),
                          SizedBox(
                            width: stickSize,
                            child: _Stick(
                              size: stickSize,
                              label: 'Pitch / Roll',
                              enabled: connected,
                              onChanged: (x, y) => ctrl.setSticks(r: x, p: y),
                              onRelease: () => ctrl.setSticks(r: 0.0, p: 0.0),
                            ),
                          ),
                        ],
                      ),
                    ),
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );
  }
}

class _TelemetryStrip extends StatelessWidget {
  const _TelemetryStrip({required this.connected, required this.text});
  final bool connected;
  final String text;

  @override
  Widget build(BuildContext context) {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
      decoration: BoxDecoration(
        color: Colors.black.withOpacity(0.04),
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: Colors.black12),
      ),
      child: Row(
        children: [
          Icon(connected ? Icons.wifi : Icons.wifi_off,
              color: connected ? Colors.green : Colors.orange),
          const SizedBox(width: 10),
          Expanded(
            child: Text(
              text,
              style: const TextStyle(fontFamily: 'monospace'),
              maxLines: 2,
              overflow: TextOverflow.ellipsis,
            ),
          ),
        ],
      ),
    );
  }
}

class _HudPanel extends StatelessWidget {
  const _HudPanel({
    required this.width,
    required this.connected,
    required this.onArm,
    required this.onDisarm,
    required this.onKill,
    required this.telemetryMultiline,
  });

  final double width;
  final bool connected;
  final VoidCallback? onArm;
  final VoidCallback? onDisarm;
  final VoidCallback? onKill;
  final List<String> telemetryMultiline;

  @override
  Widget build(BuildContext context) {
    return ConstrainedBox(
      constraints: BoxConstraints.tightFor(width: width),
      child: Card(
        elevation: 2,
        child: Padding(
          padding: const EdgeInsets.fromLTRB(12, 12, 12, 10),
          child: LayoutBuilder(
            builder: (context, c) {
              // Scroll if panel content doesn't fit (prevents overflow).
              return SingleChildScrollView(
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    if (!connected)
                      Container(
                        width: double.infinity,
                        padding: const EdgeInsets.all(8),
                        decoration: BoxDecoration(
                          color: Colors.amber.withOpacity(0.25),
                          border: Border.all(color: Colors.amber),
                          borderRadius: BorderRadius.circular(8),
                        ),
                        child: const Text(
                          'Not connected',
                          textAlign: TextAlign.center,
                        ),
                      ),
                    if (!connected) const SizedBox(height: 10),

                    Row(children: [
                      Expanded(child: ElevatedButton(onPressed: onArm, child: const Text('ARM'))),
                    ]),
                    const SizedBox(height: 8),
                    Row(children: [
                      Expanded(child: ElevatedButton(onPressed: onDisarm, child: const Text('DISARM'))),
                    ]),
                    const SizedBox(height: 8),
                    Row(children: [
                      Expanded(
                        child: ElevatedButton(
                          onPressed: onKill,
                          style: ElevatedButton.styleFrom(
                            backgroundColor: Colors.red,
                            foregroundColor: Colors.white,
                            disabledBackgroundColor: Colors.red.withOpacity(0.35),
                            disabledForegroundColor: Colors.white70,
                          ),
                          child: const Text('KILL'),
                        ),
                      ),
                    ]),
                    const SizedBox(height: 10),

                    Container(
                      width: double.infinity,
                      padding: const EdgeInsets.all(10),
                      decoration: BoxDecoration(
                        color: Colors.black.withOpacity(0.04),
                        borderRadius: BorderRadius.circular(8),
                        border: Border.all(color: Colors.black12),
                      ),
                      child: Text(
                        telemetryMultiline.join('\n'),
                        style: const TextStyle(fontFamily: 'monospace', fontSize: 14),
                      ),
                    ),
                  ],
                ),
              );
            },
          ),
        ),
      ),
    );
  }
}

class _PortraitTopPanel extends StatelessWidget {
  const _PortraitTopPanel({
    required this.connected,
    required this.onArm,
    required this.onDisarm,
    required this.onKill,
    required this.telemetry,
  });

  final bool connected;
  final VoidCallback? onArm;
  final VoidCallback? onDisarm;
  final VoidCallback? onKill;
  final String telemetry;

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        if (!connected)
          Container(
            width: double.infinity,
            padding: const EdgeInsets.all(10),
            decoration: BoxDecoration(
              color: Colors.amber.withOpacity(0.25),
              border: Border.all(color: Colors.amber),
              borderRadius: BorderRadius.circular(10),
            ),
            child: const Row(
              children: [
                Icon(Icons.wifi_off),
                SizedBox(width: 10),
                Expanded(
                  child: Text(
                    'Not connected — controls disabled. Connect in Console tab.',
                    style: TextStyle(fontSize: 14),
                  ),
                ),
              ],
            ),
          ),
        if (!connected) const SizedBox(height: 10),
        Row(
          children: [
            Expanded(child: ElevatedButton(onPressed: onArm, child: const Text('ARM'))),
            const SizedBox(width: 10),
            Expanded(child: ElevatedButton(onPressed: onDisarm, child: const Text('DISARM'))),
            const SizedBox(width: 10),
            Expanded(
              child: ElevatedButton(
                onPressed: onKill,
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.red,
                  foregroundColor: Colors.white,
                  disabledBackgroundColor: Colors.red.withOpacity(0.35),
                  disabledForegroundColor: Colors.white70,
                ),
                child: const Text('KILL'),
              ),
            ),
          ],
        ),
        const SizedBox(height: 8),
        Align(
          alignment: Alignment.centerLeft,
          child: Text(telemetry, style: const TextStyle(fontFamily: 'monospace')),
        ),
      ],
    );
  }
}

/// Joystick: easy grab + springs to center
class _Stick extends StatefulWidget {
  const _Stick({
    required this.size,
    required this.label,
    required this.enabled,
    required this.onChanged,
    required this.onRelease,
  });

  final double size;
  final String label;
  final bool enabled;
  final void Function(double x, double y) onChanged;
  final VoidCallback onRelease;

  @override
  State<_Stick> createState() => _StickState();
}

class _StickState extends State<_Stick> {
  Offset _pos = Offset.zero;
  static const double _deadzone = 0.05;

  @override
  Widget build(BuildContext context) {
    final r = widget.size / 2;
    final knobR = widget.size * 0.20;

    Offset clampToCircle(Offset p) {
      if (p.distance <= r) return p;
      final ang = atan2(p.dy, p.dx);
      return Offset(cos(ang) * r, sin(ang) * r);
    }

    void emit(Offset p) {
      final nx = (p.dx / r).clamp(-1.0, 1.0);
      final ny = (-p.dy / r).clamp(-1.0, 1.0);
      double dz(double v) => v.abs() < _deadzone ? 0.0 : v;
      widget.onChanged(dz(nx), dz(ny));
    }

    void setFromLocal(Offset local) {
      final centered = local - Offset(r, r);
      final clamped = clampToCircle(centered);
      setState(() => _pos = clamped);
      emit(_pos);
    }

    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        Text(widget.label, style: Theme.of(context).textTheme.titleMedium),
        const SizedBox(height: 8),
        Opacity(
          opacity: widget.enabled ? 1.0 : 0.45,
          child: GestureDetector(
            behavior: HitTestBehavior.opaque,
            onPanDown: widget.enabled ? (d) => setFromLocal(d.localPosition) : null,
            onPanUpdate: widget.enabled ? (d) => setFromLocal(d.localPosition) : null,
            onPanEnd: widget.enabled
                ? (_) {
                    widget.onRelease();
                    setState(() => _pos = Offset.zero);
                  }
                : null,
            child: Container(
              width: widget.size,
              height: widget.size,
              decoration: BoxDecoration(
                color: Colors.black12,
                borderRadius: BorderRadius.circular(widget.size / 2),
                border: Border.all(color: Colors.black26),
              ),
              child: Stack(
                children: [
                  Positioned(left: r - 1, top: 0, bottom: 0, child: Container(width: 2, color: Colors.black12)),
                  Positioned(top: r - 1, left: 0, right: 0, child: Container(height: 2, color: Colors.black12)),
                  Positioned(
                    left: r + _pos.dx - knobR,
                    top: r + _pos.dy - knobR,
                    child: Container(
                      width: knobR * 2,
                      height: knobR * 2,
                      decoration: BoxDecoration(
                        color: Colors.black54,
                        borderRadius: BorderRadius.circular(knobR),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ],
    );
  }
}
