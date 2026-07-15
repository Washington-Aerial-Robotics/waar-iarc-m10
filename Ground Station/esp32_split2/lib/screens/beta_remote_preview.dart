import 'dart:math';
import 'package:flutter/material.dart';

/// Beta Remote screen for experimenting with the controller UI.
///
/// Responsibilities:
/// - Mirrors the Remote Control screen layout for testing and iteration.
/// - Shows the same telemetry-style readouts driven by joystick interaction.
/// - Includes the same center HUD controls (ARM/DISARM/KILL) for UI parity.
/// - Does not need an active drone connection; behavior can be simulated.
///
/// This screen is intended as a sandbox for new UI ideas before merging them
/// into the production Remote Control screen.
/// NO LONGER IN APP UI AS OF 4/16/2026

class BetaRemotePreview extends StatefulWidget {
  const BetaRemotePreview({super.key});

  @override
  State<BetaRemotePreview> createState() => _BetaRemotePreviewState();
}

class _BetaRemotePreviewState extends State<BetaRemotePreview> {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double throttle = 0.0;

  void setSticks({double? r, double? p, double? y, double? t}) {
    setState(() {
      if (r != null) roll = r.clamp(-1.0, 1.0);
      if (p != null) pitch = p.clamp(-1.0, 1.0);
      if (y != null) yaw = y.clamp(-1.0, 1.0);
      if (t != null) throttle = t.clamp(0.0, 1.0);
    });
  }

  @override
  Widget build(BuildContext context) {
    final isLandscape =
        MediaQuery.of(context).orientation == Orientation.landscape;

    // In case this page sits under a parent BottomNavigationBar
    final bottomNavClearance =
        kBottomNavigationBarHeight + MediaQuery.of(context).padding.bottom;

    String telemetryInline() =>
        'thr=${throttle.toStringAsFixed(2)}   '
        'yaw=${yaw.toStringAsFixed(2)}   '
        'pit=${pitch.toStringAsFixed(2)}   '
        'rol=${roll.toStringAsFixed(2)}';

    List<String> telemetryLines() => [
          'thr=${throttle.toStringAsFixed(2)}',
          'yaw=${yaw.toStringAsFixed(2)}',
          'pit=${pitch.toStringAsFixed(2)}',
          'rol=${roll.toStringAsFixed(2)}',
        ];

    return Scaffold(
      appBar: AppBar(
        title: const Text('Beta Remote'),
        centerTitle: true,
      ),
      body: SafeArea(
        child: LayoutBuilder(
          builder: (context, c) {
            const pad = 8.0;

            if (isLandscape) {
              final topH = (c.maxHeight * 0.26).clamp(86.0, 130.0);
              final bottomH = max(
                0.0,
                c.maxHeight - topH - pad - bottomNavClearance,
              );

              final hudW = (c.maxWidth * 0.22).clamp(210.0, 300.0);

              final availableW = max(0.0, c.maxWidth - hudW - pad * 2 - 16);
              final eachStickW = availableW / 2;

              final stickSize =
                  min(eachStickW, bottomH - 34).clamp(150.0, 460.0);

              return Padding(
                padding: EdgeInsets.fromLTRB(
                  pad,
                  pad,
                  pad,
                  pad + bottomNavClearance,
                ),
                child: Column(
                  children: [
                    _TelemetryStrip(connected: true, text: telemetryInline()),
                    const SizedBox(height: 8),
                    Expanded(
                      child: Row(
                        crossAxisAlignment: CrossAxisAlignment.end,
                        children: [
                          SizedBox(
                            width: stickSize,
                            child: _Stick(
                              size: stickSize,
                              label: 'Throttle / Yaw',
                              onChanged: (x, y) {
                                final thr = ((y + 1) / 2).clamp(0.0, 1.0);
                                setSticks(t: thr, y: x);
                              },
                              onRelease: () => setSticks(t: 0.0, y: 0.0),
                            ),
                          ),
                          const SizedBox(width: 8),
                          Expanded(
                            child: Center(
                              child: _HudPanelWithButtons(
                                width: hudW,
                                telemetry: telemetryLines(),
                              ),
                            ),
                          ),
                          const SizedBox(width: 8),
                          SizedBox(
                            width: stickSize,
                            child: _Stick(
                              size: stickSize,
                              label: 'Pitch / Roll',
                              onChanged: (x, y) => setSticks(r: x, p: y),
                              onRelease: () => setSticks(r: 0.0, p: 0.0),
                            ),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              );
            }

            // Portrait: telemetry top + sticks below
            final stickSize = min(
              (c.maxWidth - pad * 2 - 12) / 2,
              c.maxHeight - 140 - bottomNavClearance,
            ).clamp(170.0, 420.0);

            return Padding(
              padding: EdgeInsets.fromLTRB(pad, pad, pad, pad + bottomNavClearance),
              child: Column(
                children: [
                  _TelemetryStrip(connected: true, text: telemetryInline()),
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
                              onChanged: (x, y) {
                                final thr = ((y + 1) / 2).clamp(0.0, 1.0);
                                setSticks(t: thr, y: x);
                              },
                              onRelease: () => setSticks(t: 0.0, y: 0.0),
                            ),
                          ),
                          SizedBox(
                            width: stickSize,
                            child: _Stick(
                              size: stickSize,
                              label: 'Pitch / Roll',
                              onChanged: (x, y) => setSticks(r: x, p: y),
                              onRelease: () => setSticks(r: 0.0, p: 0.0),
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
          Icon(
            connected ? Icons.wifi : Icons.wifi_off,
            color: connected ? Colors.green : Colors.orange,
          ),
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

class _HudPanelWithButtons extends StatelessWidget {
  const _HudPanelWithButtons({
    required this.width,
    required this.telemetry,
  });

  final double width;
  final List<String> telemetry;

  @override
  Widget build(BuildContext context) {
    return ConstrainedBox(
      constraints: BoxConstraints.tightFor(width: width),
      child: Card(
        elevation: 2,
        child: Padding(
          padding: const EdgeInsets.fromLTRB(12, 12, 12, 10),
          child: SingleChildScrollView(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                // Buttons (disabled in Beta)
                Row(children: [
                  Expanded(child: ElevatedButton(onPressed: null, child: const Text('ARM'))),
                ]),
                const SizedBox(height: 8),
                Row(children: [
                  Expanded(child: ElevatedButton(onPressed: null, child: const Text('DISARM'))),
                ]),
                const SizedBox(height: 8),
                Row(children: [
                  Expanded(
                    child: ElevatedButton(
                      onPressed: null,
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

                // Telemetry box
                Container(
                  width: double.infinity,
                  padding: const EdgeInsets.all(10),
                  decoration: BoxDecoration(
                    color: Colors.black.withOpacity(0.04),
                    borderRadius: BorderRadius.circular(8),
                    border: Border.all(color: Colors.black12),
                  ),
                  child: Text(
                    telemetry.join('\n'),
                    style: const TextStyle(fontFamily: 'monospace', fontSize: 14),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

/// Stick: easy grab + springs to center
class _Stick extends StatefulWidget {
  const _Stick({
    required this.size,
    required this.label,
    required this.onChanged,
    required this.onRelease,
  });

  final double size;
  final String label;
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
        GestureDetector(
          behavior: HitTestBehavior.opaque,
          onPanDown: (d) => setFromLocal(d.localPosition),
          onPanUpdate: (d) => setFromLocal(d.localPosition),
          onPanEnd: (_) {
            widget.onRelease();
            setState(() => _pos = Offset.zero);
          },
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
      ],
    );
  }
}
