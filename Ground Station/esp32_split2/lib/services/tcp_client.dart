import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart';

/// TCP networking client for communication with the ESP32.

class TcpClient {
  Socket? _sock;
  int _generation = 0;
  final _connectionCtrl = StreamController<bool>.broadcast();
  Stream<bool> get connectionChanges => _connectionCtrl.stream;
  final _linesCtrl = StreamController<String>.broadcast();
  final _bytesCtrl = StreamController<Uint8List>.broadcast();

  String _rxBuffer = '';

  bool get isConnected => _sock != null;
  Stream<String> get lines => _linesCtrl.stream;
  Stream<Uint8List> get packets => _bytesCtrl.stream;

  Future<void> connect(
    String ip,
    int port, {
    Duration timeout = const Duration(seconds: 5),
  }) async {
    if (_sock != null) return;

    final generation = ++_generation;
    final sock = await Socket.connect(ip, port, timeout: timeout);
    if (generation != _generation) {
      sock.destroy();
      throw StateError("Connection cancelled");
    }
    _sock = sock;
    _connectionCtrl.add(true);

    sock.listen(
      (Uint8List data) {
        if (!identical(_sock, sock)) return;
        // RAW BYTES LOG
        final hex = data
            .map((b) => b.toRadixString(16).padLeft(2, '0').toUpperCase())
            .join(' ');
        debugPrint('APP RX [$hex]');

        _bytesCtrl.add(Uint8List.fromList(data));

        final text = String.fromCharCodes(data);
        final combined = _rxBuffer + text;
        final parts = combined.split('\n');

        for (int i = 0; i < parts.length - 1; i++) {
          _linesCtrl.add(parts[i].replaceAll('\r', ''));
        }
        _rxBuffer = parts.last;
        if (_rxBuffer.length > 4096) {
          _rxBuffer = ''; // Bound legacy text logging.
        }
      },
      onDone: () {
        if (!identical(_sock, sock)) return;
        if (_rxBuffer.isNotEmpty) {
          _linesCtrl.add(_rxBuffer.replaceAll('\r', ''));
          _rxBuffer = '';
        }
        disconnect();
        _linesCtrl.add('Disconnected (remote).');
      },
      onError: (e) {
        if (!identical(_sock, sock)) return;
        _linesCtrl.add('Error: $e');
        disconnect();
      },
    );
  }

  void sendBytes(List<int> bytes) {
    if (_sock == null) {
      debugPrint('APP TX FAILED (no socket)');
      return;
    }

    // RAW TX LOG
    final hex = bytes
        .map((b) => b.toRadixString(16).padLeft(2, '0').toUpperCase())
        .join(' ');
    debugPrint('APP TX [$hex]');

    _sock!.add(bytes);
    // Socket.add queues bytes; socket errors are handled by the listener.
  }

  void sendLine(String text) {
    final payload = text.endsWith('\n') ? text : '$text\n';
    _sock?.add(utf8.encode(payload));
  }

  void disconnect() {
    ++_generation;
    final wasConnected = _sock != null;
    _sock?.destroy();
    _sock = null;
    _rxBuffer = '';
    if (wasConnected) _connectionCtrl.add(false);
  }

  void dispose() {
    disconnect();
    _linesCtrl.close();
    _bytesCtrl.close();
    _connectionCtrl.close();
  }
}
