import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/foundation.dart';

import '../protocol/drone_protocol.dart';
import '../services/tcp_client.dart';

enum LinkStatus { disconnected, connecting, connected, error }

class DroneController extends ChangeNotifier {
  DroneController(this.client)
    : _packetBuilder = DronePacketBuilder(
        fromId: appDeviceId,
        defaultToId: defaultDroneDeviceId,
      ) {
    _packetSub = client.packets.listen(_receiveBytes);
    _connectionSub = client.connectionChanges.listen((connected) {
      if (!connected) {
        _resetControls();
        status = LinkStatus.disconnected;
        _pushLog('Link lost — controls reset; ARM required after reconnect');
      }
    });
  }

  final TcpClient client;
  final DronePacketBuilder _packetBuilder;

  static const int appDeviceId = 0x47; // 'G'
  static const int defaultDroneDeviceId = 0x41; // 'A'

  int _targetDroneId = defaultDroneDeviceId;

  int get targetDroneId => _targetDroneId;

  String get targetDroneCharacter {
    return String.fromCharCode(_targetDroneId);
  }

  LinkStatus status = LinkStatus.disconnected;
  String? lastError;

  final List<String> log = [];
  StreamSubscription<String>? _sub;
  StreamSubscription<bool>? _connectionSub;
  int _connectGeneration = 0;
  int _controlGeneration = 0;
  bool _disposed = false;
  bool controlPending = false;
  final _decoder = KentReplyDecoder();
  StreamSubscription<Uint8List>? _packetSub;
  Completer<DronePacket>? _pending;
  int? _pendingId;
  int? _expectedType;

  void _receiveBytes(Uint8List bytes) {
    try {
      for (final packet in _decoder.add(bytes)) {
        if (packet.toId != appDeviceId || packet.fromId != targetDroneId) {
          continue;
        }
        if (packet.messageId == _pendingId && _pending != null) {
          if (packet.messageType == _expectedType) {
            _pending!.complete(packet);
          } else {
            _pending!.completeError(
              StateError('Kent rejected command: ${packet.messageType}'),
            );
          }
          _pending = null;
        }
      }
    } catch (error) {
      _pushLog('Protocol error: $error');
      client.disconnect();
      _resetControls();
    }
  }

  Future<DronePacket> sendRequest(
    Uint8List packet, {
    int expectedType = DroneComms.comSuccess,
  }) async {
    if (!client.isConnected || _pending != null) {
      throw StateError('Disconnected or command pending');
    }
    final completer = Completer<DronePacket>();
    _pending = completer;
    _pendingId = packet[3];
    _expectedType = expectedType;
    try {
      client.sendBytes(packet);
      return await completer.future.timeout(const Duration(seconds: 2));
    } finally {
      if (identical(_pending, completer)) _pending = null;
    }
  }

  Future<void> requestDiagnostic(int type, int replyType) async {
    try {
      final reply = await sendRequest(
        _packetBuilder.headerOnly(messageType: type, toId: targetDroneId),
        expectedType: replyType,
      );
      if (replyType == DroneComms.comReplyPos) {
        final data = ByteData.sublistView(reply.payload);
        _pushLog(
          'STATE x=${data.getFloat32(0, Endian.little)} y=${data.getFloat32(4, Endian.little)} z=${data.getFloat32(8, Endian.little)} mode=${reply.payload[12]}',
        );
      } else if (replyType == DroneComms.comReplyWifi) {
        _pushLog(
          'Wi-Fi IP: ${String.fromCharCodes(reply.payload.takeWhile((b) => b != 0))}',
        );
      } else {
        _pushLog('Kent ping confirmed');
      }
    } catch (error) {
      _pushLog('Request failed: $error');
    }
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double throttle = 0.0;

  Timer? _txTimer;
  Timer? _voiceResetTimer;

  bool _killed = false;

  bool get isConnected {
    return client.isConnected && status == LinkStatus.connected;
  }

  bool get isConnecting {
    return status == LinkStatus.connecting;
  }

  bool get isControlLoopRunning {
    return _txTimer != null;
  }

  static const double voiceThrottleStep = 0.08;
  static const double voiceMovementAmount = 0.25;
  static const double voiceYawAmount = 0.25;

  // Directional voice commands automatically center after this time.
  static const Duration voiceMovementDuration = Duration(milliseconds: 750);

  void _pushLog(String message) {
    if (_disposed) return;
    log.insert(0, message);

    if (log.length > 300) {
      log.removeLast();
    }

    notifyListeners();
  }

  bool setTargetDroneId(String input) {
    final normalized = input.trim().toUpperCase();

    if (normalized.length != 1) {
      _pushLog('Drone ID must be exactly one character');
      return false;
    }

    final codeUnit = normalized.codeUnitAt(0);

    if (codeUnit < 0x21 || codeUnit > 0x7E) {
      _pushLog('Drone ID must be a printable character');
      return false;
    }

    if (client.isConnected) {
      _pushLog('Disconnect before changing drone ID');
      return false;
    }
    _targetDroneId = codeUnit;

    _pushLog(
      'Target drone changed to '
      '$targetDroneCharacter '
      '(0x${_targetDroneId.toRadixString(16).padLeft(2, '0').toUpperCase()})',
    );

    notifyListeners();
    return true;
  }

  Future<void> connect(String ip, int port) async {
    if (isConnected || isConnecting) {
      return;
    }

    final generation = ++_connectGeneration;
    _resetControls();
    status = LinkStatus.connecting;
    lastError = null;
    _killed = false;

    notifyListeners();
    _decoder.clear();
    _pushLog('Connecting to $ip:$port…');

    try {
      await client.connect(ip, port);

      if (generation != _connectGeneration) return;
      status = LinkStatus.connected;
      _pushLog('Connected');

      await _sub?.cancel();
      _sub = client.lines.listen(_pushLog);

      // Do not begin sending motor packets immediately after connection.
      notifyListeners();
    } catch (error) {
      if (generation != _connectGeneration) return;
      lastError = error.toString();
      status = LinkStatus.error;

      _pushLog('Connection failed: $error');
      notifyListeners();
    }
  }

  Future<void> disconnect() async {
    ++_connectGeneration;
    _resetControls();
    client.disconnect();

    _voiceResetTimer?.cancel();
    _voiceResetTimer = null;

    await _sub?.cancel();
    _sub = null;

    client.disconnect();

    status = LinkStatus.disconnected;
    notifyListeners();
  }

  void setSticks({double? r, double? p, double? y, double? t}) {
    _voiceResetTimer?.cancel();
    _voiceResetTimer = null;

    if (r != null) {
      roll = r.clamp(-1.0, 1.0);
    }

    if (p != null) {
      pitch = p.clamp(-1.0, 1.0);
    }

    if (y != null) {
      yaw = y.clamp(-1.0, 1.0);
    }

    if (t != null) {
      throttle = t.clamp(0.0, 1.0);
    }

    notifyListeners();
  }

  Future<void> arm() async {
    if (!isConnected || _killed || controlPending || isControlLoopRunning) {
      return;
    }
    _resetControls();
    final generation = _controlGeneration;
    controlPending = true;
    notifyListeners();
    try {
      await sendRequest(
        _packetBuilder.singleByte(
          messageType: DroneComms.comSetActuation,
          value: 0,
          toId: targetDroneId,
        ),
      );
      if (generation != _controlGeneration) return;
      await sendRequest(
        _packetBuilder.setpoints(
          [0, 0, 0, 0],
          mode: DroneComms.motorSetpointMode,
          toId: targetDroneId,
        ),
      );
      if (generation != _controlGeneration) return;
      await sendRequest(
        _packetBuilder.singleByte(
          messageType: DroneComms.comSetActuation,
          value: 255,
          toId: targetDroneId,
        ),
      );
      if (generation != _controlGeneration) return;
      _startControlLoop();
      _pushLog('Kent accepted motor mode and actuation enable — bench control');
    } catch (error) {
      if (generation != _controlGeneration) return;
      _resetControls();
      _killed = true;
      _pushLog('ARM failed: $error; actuation state unconfirmed');
    } finally {
      if (!_disposed && (generation == _controlGeneration || _killed)) {
        controlPending = false;
        notifyListeners();
      }
    }
  }

  Future<void> disarm() async {
    _resetControls();
    final generation = _controlGeneration;
    if (!isConnected) return;
    controlPending = true;
    notifyListeners();
    try {
      await sendRequest(
        _packetBuilder.singleByte(
          messageType: DroneComms.comSetActuation,
          value: 0,
          toId: targetDroneId,
        ),
      );
      if (generation != _controlGeneration) return;
      await sendRequest(
        _packetBuilder.setpoints(
          [],
          mode: DroneComms.nullMode,
          toId: targetDroneId,
        ),
      );
      _pushLog('Kent accepted actuation disable and null mode');
    } catch (error) {
      _pushLog('DISARM unconfirmed: $error');
    } finally {
      if (!_disposed && (generation == _controlGeneration || _killed)) {
        controlPending = false;
        notifyListeners();
      }
    }
  }

  void kill() {
    if (!client.isConnected) {
      _pushLog('Cannot KILL: not connected');
      return;
    }

    _voiceResetTimer?.cancel();
    _voiceResetTimer = null;

    _resetControls();
    _killed = true;
    _stopControlLoop();

    throttle = 0.0;
    pitch = 0.0;
    roll = 0.0;
    yaw = 0.0;

    final packet = _packetBuilder.headerOnly(
      messageType: DroneComms.comKill,
      toId: _targetDroneId,
    );

    client.sendBytes(packet);

    _pushLog('Sent KILL to $targetDroneCharacter');
    notifyListeners();
  }

  void startControlLoop() {
    arm();
  }

  void stopControlLoop() {
    _stopControlLoop();
    _pushLog('Control loop stopped');
  }

  bool _canAcceptVoiceCommand(String command) {
    if (!client.isConnected) {
      _pushLog('Voice $command ignored: not connected');
      return false;
    }

    if (_killed) {
      _pushLog('Voice $command ignored: drone is killed');
      return false;
    }

    if (_txTimer == null) {
      _pushLog('Voice $command ignored: drone is not armed');
      return false;
    }

    return true;
  }

  void voiceUp() {
    if (!_canAcceptVoiceCommand('UP')) {
      return;
    }

    throttle = (throttle + voiceThrottleStep).clamp(0.0, 1.0);

    _pushLog('Voice: UP — throttle=${throttle.toStringAsFixed(2)}');

    notifyListeners();
  }

  void voiceDown() {
    if (!_canAcceptVoiceCommand('DOWN')) {
      return;
    }

    throttle = (throttle - voiceThrottleStep).clamp(0.0, 1.0);

    _pushLog('Voice: DOWN — throttle=${throttle.toStringAsFixed(2)}');

    notifyListeners();
  }

  void voiceHover() {
    if (!_canAcceptVoiceCommand('HOVER')) {
      return;
    }

    _pushLog('HOVER unavailable: onboard altitude hold is not implemented');
  }

  void voiceStop() {
    if (!_canAcceptVoiceCommand('STOP')) {
      return;
    }

    _voiceResetTimer?.cancel();
    _voiceResetTimer = null;

    // Keep the current throttle so the drone does not suddenly fall.
    // Stop only translation and rotation.
    pitch = 0.0;
    roll = 0.0;
    yaw = 0.0;

    _pushLog('Voice: STOP — directional controls centered');
    notifyListeners();
  }

  void voiceForward() {
    if (!_canAcceptVoiceCommand('FORWARD')) {
      return;
    }

    pitch = voiceMovementAmount;
    roll = 0.0;
    yaw = 0.0;

    _scheduleVoiceDirectionalReset();
    _pushLog('Voice: FORWARD');

    notifyListeners();
  }

  void voiceBackward() {
    if (!_canAcceptVoiceCommand('BACKWARD')) {
      return;
    }

    pitch = -voiceMovementAmount;
    roll = 0.0;
    yaw = 0.0;

    _scheduleVoiceDirectionalReset();
    _pushLog('Voice: BACKWARD');

    notifyListeners();
  }

  void voiceMoveLeft() {
    if (!_canAcceptVoiceCommand('MOVE LEFT')) {
      return;
    }

    pitch = 0.0;
    roll = -voiceMovementAmount;
    yaw = 0.0;

    _scheduleVoiceDirectionalReset();
    _pushLog('Voice: MOVE LEFT');

    notifyListeners();
  }

  void voiceMoveRight() {
    if (!_canAcceptVoiceCommand('MOVE RIGHT')) {
      return;
    }

    pitch = 0.0;
    roll = voiceMovementAmount;
    yaw = 0.0;

    _scheduleVoiceDirectionalReset();
    _pushLog('Voice: MOVE RIGHT');

    notifyListeners();
  }

  void voiceRotateLeft() {
    if (!_canAcceptVoiceCommand('ROTATE LEFT')) {
      return;
    }

    pitch = 0.0;
    roll = 0.0;
    yaw = -voiceYawAmount;

    _scheduleVoiceDirectionalReset();
    _pushLog('Voice: ROTATE LEFT');

    notifyListeners();
  }

  void voiceRotateRight() {
    if (!_canAcceptVoiceCommand('ROTATE RIGHT')) {
      return;
    }

    pitch = 0.0;
    roll = 0.0;
    yaw = voiceYawAmount;

    _scheduleVoiceDirectionalReset();
    _pushLog('Voice: ROTATE RIGHT');

    notifyListeners();
  }

  void _scheduleVoiceDirectionalReset() {
    _voiceResetTimer?.cancel();

    _voiceResetTimer = Timer(voiceMovementDuration, () {
      pitch = 0.0;
      roll = 0.0;
      yaw = 0.0;

      _pushLog('Voice movement completed — controls centered');
      notifyListeners();
    });
  }

  double _curveSigned(double value) {
    final clamped = value.clamp(-1.0, 1.0);

    if (clamped == 0.0) {
      return 0.0;
    }

    final sign = clamped < 0 ? -1.0 : 1.0;
    return sign * math.sqrt(clamped.abs());
  }

  double _curveThrottle(double value) {
    return math.sqrt(value.clamp(0.0, 1.0));
  }

  double _clampMotor(double value) {
    return value.clamp(0.0, 1.0);
  }

  void _startControlLoop() {
    _txTimer?.cancel();

    _txTimer = Timer.periodic(const Duration(milliseconds: 250), (_) {
      if (!client.isConnected || _killed) {
        return;
      }

      final shapedThrottle = _curveThrottle(throttle);
      final shapedPitch = _curveSigned(pitch);
      final shapedRoll = _curveSigned(roll);
      final shapedYaw = _curveSigned(yaw);

      final motor0 = _clampMotor(
        shapedThrottle + shapedPitch + shapedRoll - shapedYaw,
      );

      final motor1 = _clampMotor(
        shapedThrottle + shapedPitch - shapedRoll + shapedYaw,
      );

      final motor2 = _clampMotor(
        shapedThrottle - shapedPitch + shapedRoll + shapedYaw,
      );

      final motor3 = _clampMotor(
        shapedThrottle - shapedPitch - shapedRoll - shapedYaw,
      );

      if (_pending != null) return;
      final packet = _packetBuilder.setpoints([
        motor0,
        motor1,
        motor2,
        motor3,
      ], toId: targetDroneId);
      final generation = _controlGeneration;
      unawaited(
        sendRequest(packet).catchError((Object error) {
          if (generation == _controlGeneration) {
            _resetControls();
            _killed = true;
            _pushLog(
              'Control update unconfirmed: $error; stopped; reconnect required',
            );
          }
          return DronePacket(
            toId: 0,
            fromId: 0,
            messageType: 0,
            messageId: 0,
            payload: Uint8List(0),
          );
        }),
      );

      debugPrint(
        'TX control pkt '
        'thr=${throttle.toStringAsFixed(2)} '
        'pit=${pitch.toStringAsFixed(2)} '
        'rol=${roll.toStringAsFixed(2)} '
        'yaw=${yaw.toStringAsFixed(2)} '
        'm0=${motor0.toStringAsFixed(2)} '
        'm1=${motor1.toStringAsFixed(2)} '
        'm2=${motor2.toStringAsFixed(2)} '
        'm3=${motor3.toStringAsFixed(2)}',
      );
    });
  }

  void _resetControls() {
    ++_controlGeneration;
    controlPending = false;
    final pending = _pending;
    _pending = null;
    if (pending != null && !pending.isCompleted) {
      pending.completeError(StateError('Control operation cancelled'));
    }
    _stopControlLoop();
    _voiceResetTimer?.cancel();
    _voiceResetTimer = null;
    throttle = pitch = roll = yaw = 0.0;
  }

  void _stopControlLoop() {
    _txTimer?.cancel();
    _txTimer = null;
  }

  @override
  void dispose() {
    _disposed = true;
    _voiceResetTimer?.cancel();
    _stopControlLoop();
    _sub?.cancel();
    _connectionSub?.cancel();
    _packetSub?.cancel();
    _resetControls();

    // TcpClient is owned and disposed by Provider in main.dart.
    super.dispose();
  }
}
