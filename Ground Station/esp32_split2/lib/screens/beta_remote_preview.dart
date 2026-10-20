import 'dart:math';
import 'package:flutter/material.dart';

/// Beta Remote screen for experimenting with the controller UI.
///
/// Responsibilities:
/// - Provides two movable joysticks in portrait and landscape.
/// - Shows the same telemetry-style readouts driven by joystick interaction.
/// - Holds all values locally, independently of the live controller.
/// - Does not need an active drone connection; behavior can be simulated.
///
/// This screen is intended as a sandbox for new UI ideas before merging them
/// into the production Remote Control screen.
/// Local preview only: never sends commands to a drone.

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
    return Scaffold(
      appBar: AppBar(title: const Text('Beta joystick preview')),
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(12),
          child: Column(
            children: [
              const Text('Local preview — no drone commands are sent.'),
              const SizedBox(height: 8),
              Text(
                'thr=${throttle.toStringAsFixed(2)}  yaw=${yaw.toStringAsFixed(2)}  '
                'pit=${pitch.toStringAsFixed(2)}  rol=${roll.toStringAsFixed(2)}',
                style: const TextStyle(fontFamily: 'monospace'),
              ),
              const SizedBox(height: 12),
              Expanded(
                child: LayoutBuilder(
                  builder: (context, bounds) {
                    final labelHeight =
                        MediaQuery.textScalerOf(context).scale(20) + 12;
                    final size = max(
                      1.0,
                      min(
                        320.0,
                        min(
                          (bounds.maxWidth - 20) / 2,
                          bounds.maxHeight - labelHeight,
                        ),
                      ),
                    );
                    return Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      crossAxisAlignment: CrossAxisAlignment.center,
                      children: [
                        _Stick(
                          size: size,
                          label: 'Throttle / Yaw',
                          onChanged: (x, y) => setSticks(t: (y + 1) / 2, y: x),
                          onRelease: () => setSticks(t: 0, y: 0),
                        ),
                        _Stick(
                          size: size,
                          label: 'Pitch / Roll',
                          onChanged: (x, y) => setSticks(r: x, p: y),
                          onRelease: () => setSticks(r: 0, p: 0),
                        ),
                      ],
                    );
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

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
          onPanCancel: () {
            widget.onRelease();
            setState(() => _pos = Offset.zero);
          },
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
                Positioned(
                  left: r - 1,
                  top: 0,
                  bottom: 0,
                  child: Container(width: 2, color: Colors.black12),
                ),
                Positioned(
                  top: r - 1,
                  left: 0,
                  right: 0,
                  child: Container(height: 2, color: Colors.black12),
                ),
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
