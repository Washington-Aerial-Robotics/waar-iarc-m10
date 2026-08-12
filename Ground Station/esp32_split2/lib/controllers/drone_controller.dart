import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/foundation.dart';

import '../protocol/drone_protocol.dart';
import '../services/tcp_client.dart';

enum LinkStatus {
  disconnected,
  connecting,
  connected,
  error,
}

class DroneController extends ChangeNotifier {
  DroneController(this.client)
      : _packetBuilder = DronePacketBuilder(
          fromId: appDeviceId,
          defaultToId: defaultDroneDeviceId,
        );

  final TcpClient client;
  final DronePacketBuilder _packetBuilder;

  static const int appDeviceId = 0x47; // 'G'
  // firmware_full.ino identifies this airframe as 'U'. Packets addressed to
  // another device ID are intentionally ignored by the firmware.
  static const int defaultDroneDeviceId = 0x55; // 'U'

  int _targetDroneId = defaultDroneDeviceId;

  int get targetDroneId => _targetDroneId;

  String get targetDroneCharacter {
    return String.fromCharCode(_targetDroneId);
  }

  LinkStatus status = LinkStatus.disconnected;
  String? lastError;

  final List<String> log = [];
  StreamSubscription<String>? _sub;
  StreamSubscription<Uint8List>? _packetSub;

  DroneTelemetry telemetry = const DroneTelemetry();

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

  // This needs to be calibrated for the actual drone.
  static const double defaultHoverThrottle = 0.50;

  // Directional voice commands automatically center after this time.
  static const Duration voiceMovementDuration =
      Duration(milliseconds: 750);

  void _pushLog(String message) {
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

    status = LinkStatus.connecting;
    lastError = null;
    _killed = false;

    notifyListeners();
    _pushLog('Connecting to $ip:$port…');

    try {
      await client.connect(ip, port);

      status = LinkStatus.connected;
      _pushLog('Connected');

      await _sub?.cancel();
      _sub = client.lines.listen(_pushLog);

      await _packetSub?.cancel();
      _packetSub = client.packets.listen(_handleIncomingPacket);

      // Do not begin sending motor packets immediately after connection.
      notifyListeners();
    } catch (error) {
      lastError = error.toString();
      status = LinkStatus.error;

      _pushLog('Connection failed: $error');
      notifyListeners();
    }
  }

  Future<void> disconnect() async {
    _stopControlLoop();

    _voiceResetTimer?.cancel();
    _voiceResetTimer = null;

    await _sub?.cancel();
    _sub = null;

    await _packetSub?.cancel();
    _packetSub = null;

    client.disconnect();

    status = LinkStatus.disconnected;
    notifyListeners();
  }

  void _handleIncomingPacket(Uint8List bytes) {
    final packet = DronePacketBuilder.tryParse(bytes);
    if (packet == null) {
      return;
    }

    DroneTelemetry? update;

    switch (packet.messageType) {
      case DroneComms.comReplyPos:
        update = DroneTelemetry.fromStateReply(packet.payload);
        break;
      case DroneComms.comReplyStateEst:
        update = DroneTelemetry.fromStateEstimateReply(packet.payload);
        break;
      case DroneComms.comReplyInfo:
        update = DroneTelemetry.fromInfoReply(packet.payload);
        break;
    }

    if (update == null) {
      return;
    }

    telemetry = telemetry.mergedWith(update);
    _pushLog('Telemetry: $telemetry');
  }

  void setSticks({
    double? r,
    double? p,
    double? y,
    double? t,
  }) {
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

  // Firmware's com_step() (communication.cpp) reads all currently-available
  // TCP bytes into one buffer and processes only the first packet in it, with
  // no leftover-byte carryover - two sendBytes() calls with no gap can land
  // in the same read and silently drop the second packet. This delay is a
  // stopgap to keep the two sends in separate TCP reads; the real fix is
  // proper per-packet framing in wifiReceiving()/com_step() on the firmware
  // side.
  static const _packetGap = Duration(milliseconds: 60);

  Future<void> arm() async {
    if (!client.isConnected) {
      _pushLog('Cannot ARM: not connected');
      return;
    }

    if (_killed) {
      _pushLog('Cannot ARM: reconnect after KILL');
      return;
    }

    // Switch the firmware out of NULL_MODE (its boot default, which zeroes
    // cmd.motors and forces actuation off every flight-loop tick) into
    // ACTUATION_MODE, the one mode flight_step() leaves cmd.motors alone in -
    // so the COM_SET_MOTORS stream from the control loop below actually
    // reaches the ESCs. CMD_NULL_MODE keeps the commander state machine
    // (storage replay, failsafes) out of the way for a manual bench test.
    final modePacket = _packetBuilder.flightMode(
      cmdMode: DroneComms.cmdModeNull,
      mode: DroneComms.flightModeActuation,
      toId: _targetDroneId,
    );
    client.sendBytes(modePacket);
    await Future.delayed(_packetGap);

    final actuationPacket = _packetBuilder.actuation(
      armed: true,
      toId: _targetDroneId,
    );
    client.sendBytes(actuationPacket);

    if (_txTimer == null) {
      _startControlLoop();
    }

    _pushLog('ARM: sent flight mode + actuation, control loop started');
  }

  Future<void> disarm() async {
    _voiceResetTimer?.cancel();
    _voiceResetTimer = null;

    throttle = 0.0;
    pitch = 0.0;
    roll = 0.0;
    yaw = 0.0;

    _stopControlLoop();

    if (client.isConnected) {
      client.sendBytes(_packetBuilder.actuation(armed: false, toId: _targetDroneId));
      await Future.delayed(_packetGap);
      // Also return to NULL_MODE so flight_step() actively re-zeroes
      // cmd.motors every tick as a second line of defense, not just whatever
      // relied on the actuation flag alone.
      client.sendBytes(_packetBuilder.flightMode(
        cmdMode: DroneComms.cmdModeNull,
        mode: DroneComms.flightModeNull,
        toId: _targetDroneId,
      ));
    }

    _pushLog('DISARM: controls zeroed, actuation off');
    notifyListeners();
  }

  void kill() {
    if (!client.isConnected) {
      _pushLog('Cannot KILL: not connected');
      return;
    }

    _voiceResetTimer?.cancel();
    _voiceResetTimer = null;

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
    if (!client.isConnected) {
      _pushLog('Cannot start control loop: not connected');
      return;
    }

    if (_killed) {
      _pushLog('Cannot start control loop: drone is killed');
      return;
    }

    if (_txTimer != null) {
      return;
    }

    _startControlLoop();
    _pushLog('Control loop started');
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

    _pushLog(
      'Voice: UP — throttle=${throttle.toStringAsFixed(2)}',
    );

    notifyListeners();
  }

  void voiceDown() {
    if (!_canAcceptVoiceCommand('DOWN')) {
      return;
    }

    throttle = (throttle - voiceThrottleStep).clamp(0.0, 1.0);

    _pushLog(
      'Voice: DOWN — throttle=${throttle.toStringAsFixed(2)}',
    );

    notifyListeners();
  }

  void voiceHover() {
    if (!_canAcceptVoiceCommand('HOVER')) {
      return;
    }

    _voiceResetTimer?.cancel();
    _voiceResetTimer = null;

    throttle = defaultHoverThrottle;
    pitch = 0.0;
    roll = 0.0;
    yaw = 0.0;

    _pushLog(
      'Voice: HOVER — throttle=${throttle.toStringAsFixed(2)}',
    );

    notifyListeners();
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

    _voiceResetTimer = Timer(
      voiceMovementDuration,
      () {
        pitch = 0.0;
        roll = 0.0;
        yaw = 0.0;

        _pushLog('Voice movement completed — controls centered');
        notifyListeners();
      },
    );
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

    _txTimer = Timer.periodic(
      const Duration(milliseconds: 250),
      (_) {
        if (!client.isConnected || _killed) {
          return;
        }

        final shapedThrottle = _curveThrottle(throttle);
        final shapedPitch = _curveSigned(pitch);
        final shapedRoll = _curveSigned(roll);
        final shapedYaw = _curveSigned(yaw);

        final motor0 = _clampMotor(
          shapedThrottle +
              shapedRoll -
              shapedPitch -
              shapedYaw,
        );

        final motor1 = _clampMotor(
          shapedThrottle +
              shapedRoll +
              shapedPitch +
              shapedYaw,
        );

        final motor2 = _clampMotor(
          shapedThrottle -
              shapedRoll -
              shapedPitch +
              shapedYaw,
        );

        final motor3 = _clampMotor(
          shapedThrottle -
              shapedRoll +
              shapedPitch -
              shapedYaw,
        );

        final packet = _packetBuilder.motor4Floats(
          m0: motor0,
          m1: motor1,
          m2: motor2,
          m3: motor3,
          toId: _targetDroneId,
        );

        client.sendBytes(packet);

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
      },
    );
  }

  void _stopControlLoop() {
    _txTimer?.cancel();
    _txTimer = null;
  }

  @override
  void dispose() {
    _voiceResetTimer?.cancel();
    _stopControlLoop();
    _sub?.cancel();
    _packetSub?.cancel();

    // TcpClient is owned and disposed by Provider in main.dart.
    super.dispose();
  }
}
