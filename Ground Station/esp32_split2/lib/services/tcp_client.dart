import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

/// TCP networking client for communication with the ESP32.

class TcpClient {
  Socket? _sock;
  final _linesCtrl = StreamController<String>.broadcast();
  final _bytesCtrl = StreamController<Uint8List>.broadcast();

  String _rxBuffer = '';

  bool get isConnected => _sock != null;
  Stream<String> get lines => _linesCtrl.stream;
  Stream<Uint8List> get packets => _bytesCtrl.stream;

  Future<void> connect(String ip, int port,
      {Duration timeout = const Duration(seconds: 5)}) async {
    if (_sock != null) return;

    final sock = await Socket.connect(ip, port, timeout: timeout);
    _sock = sock;

    sock.listen((Uint8List data) {
      // 🔴 RAW BYTES LOG
      final hex = data
          .map((b) => b.toRadixString(16).padLeft(2, '0').toUpperCase())
          .join(' ');
      print('APP RX [$hex]');

      _bytesCtrl.add(Uint8List.fromList(data));

      final text = String.fromCharCodes(data);
      final combined = _rxBuffer + text;
      final parts = combined.split('\n');

      for (int i = 0; i < parts.length - 1; i++) {
        _linesCtrl.add(parts[i].replaceAll('\r', ''));
      }
      _rxBuffer = parts.last;
    }, onDone: () {
      if (_rxBuffer.isNotEmpty) {
        _linesCtrl.add(_rxBuffer.replaceAll('\r', ''));
        _rxBuffer = '';
      }
      disconnect();
      _linesCtrl.add('Disconnected (remote).');
    }, onError: (e) {
      _linesCtrl.add('Error: $e');
      disconnect();
    });
  }

  void sendBytes(List<int> bytes) {
    if (_sock == null) {
      print('APP TX FAILED (no socket)');
      return;
    }

    // 🔴 RAW TX LOG
    final hex = bytes
        .map((b) => b.toRadixString(16).padLeft(2, '0').toUpperCase())
        .join(' ');
    print('APP TX [$hex]');

    // Deliberately not calling flush() here: it returns an unawaited Future,
    // and back-to-back sendBytes() calls (e.g. ARM's flight-mode + actuation
    // packets) raced two overlapping flush() calls, throwing "Bad state:
    // StreamSink is bound to a stream" and breaking the socket. add() alone
    // is sufficient to push bytes out.
    _sock!.add(bytes);
  }

  void sendLine(String text) {
    final payload = text.endsWith('\n') ? text : '$text\n';
    _sock?.add(utf8.encode(payload));
  }

  void disconnect() {
    _sock?.destroy();
    _sock = null;
    _rxBuffer = '';
  }

  void dispose() {
    disconnect();
    _linesCtrl.close();
    _bytesCtrl.close();
  }
}