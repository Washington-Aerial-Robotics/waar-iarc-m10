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
  final List<int> _replyBuffer = [];
  Completer<bool>? _telemetryRequest;
  int? _requestId;
  int? _requestDroneId;
  int? _replyType;
  Timer? _requestTimer;
  bool get telemetryRequestPending => _telemetryRequest != null;

  int _manualGeneration = 0;
  Timer? _infoPollTimer;
  Timer? _missionTimer;
  bool Function(DroneTelemetry)? _missionExpected;
  Completer<bool>? _missionResult;
  String missionStatus = 'No mission command sent';
  DateTime? lastInfoAt;
  bool get missionCommandPending => _missionResult != null;
  bool get missionInfoFresh =>
      isConnected &&
      lastInfoAt != null &&
      DateTime.now().difference(lastInfoAt!) <
          const Duration(milliseconds: 1500);
  bool get sensorsReady =>
      missionInfoFresh && ((telemetry.sensorStatus ?? 0) & 15) == 15;

  void _finishMission(bool success, String message) {
    _missionTimer?.cancel();
    _missionTimer = null;
    final result = _missionResult;
    _missionResult = null;
    _missionExpected = null;
    missionStatus = message;
    result?.complete(success);
  }

  void _stopMissionTracking() {
    ++_manualGeneration;
    _infoPollTimer?.cancel();
    _infoPollTimer = null;
    lastInfoAt = null;
    _finishMission(false, 'Disconnected / mission state unknown');
  }

  void _pollInfo() {
    if (!client.isConnected) {
      _stopMissionTracking();
      _finishTelemetryRequest(false);
      _stopControlLoop();
      status = LinkStatus.disconnected;
      notifyListeners();
      return;
    }
    if (!telemetryRequestPending) requestInfo();
    notifyListeners();
  }

  Future<bool> setAutonomyMode(int mode) {
    if (mode < 0 || mode > 3 || !missionInfoFresh || telemetry.armed != false) {
      _pushLog(
        'Mode change blocked: valid mode and fresh disarmed INFO required',
      );
      return Future.value(false);
    }
    return _sendMission(
      _packetBuilder.autonomyMode(mode: mode, toId: targetDroneId),
      'Mode $mode',
      (t) => t.autonomyMode == mode,
    );
  }

  Future<bool> setFormationSlot(int slot) {
    if (slot < 0 || slot > 3 || !missionInfoFresh || telemetry.armed != false) {
      _pushLog(
        'Slot change blocked: slot 0–3 and fresh disarmed INFO required',
      );
      return Future.value(false);
    }
    return _sendMission(
      _packetBuilder.formationSlot(slot: slot, toId: targetDroneId),
      'Slot $slot',
      (t) => t.formationSlot == slot,
    );
  }

  Future<bool> sendQualCommand(int command) {
    if (command < 1 || command > 5) return Future.value(false);
    final emergency =
        command == DroneComms.qualLand || command == DroneComms.qualAbort;
    if (!emergency &&
        (!missionInfoFresh ||
            telemetry.autonomyMode != 1 ||
            (command == DroneComms.qualLaunch && !sensorsReady))) {
      _pushLog(
        'QUAL blocked: QUAL mode and fresh INFO required; LAUNCH needs GPS/BARO/IMU/MAG',
      );
      return Future.value(false);
    }
    final states = switch (command) {
      1 => [1, 2],
      2 => [3, 4],
      3 => [4],
      _ => [5, 6],
    };
    return _sendMission(
      _packetBuilder.qualCommand(command: command, toId: targetDroneId),
      'QUAL $command',
      (t) => t.autonomyMode == 1 && states.contains(t.qualState),
      emergency: emergency,
    );
  }

  Future<bool> sendSquareCommand(int command) {
    if (command < 1 || command > 3) return Future.value(false);
    final emergency = command != DroneComms.squareStart;
    if (!emergency &&
        (!missionInfoFresh || telemetry.autonomyMode != 3 || !sensorsReady)) {
      _pushLog(
        'SQUARE START blocked: SQUARE mode and GPS/BARO/IMU/MAG required',
      );
      return Future.value(false);
    }
    return _sendMission(
      _packetBuilder.squareCommand(command: command, toId: targetDroneId),
      'SQUARE $command',
      (t) =>
          t.autonomyMode == 3 &&
          (emergency
              ? [6, 7].contains(t.squareState)
              : [1, 2, 3, 4, 5].contains(t.squareState)),
      emergency: emergency,
    );
  }

  Future<bool> _sendMission(
    Uint8List packet,
    String label,
    bool Function(DroneTelemetry) expected, {
    bool emergency = false,
  }) {
    if (!isConnected || (!emergency && (_killed || missionCommandPending))) {
      _pushLog('$label blocked: disconnected, killed, or command pending');
      return Future.value(false);
    }
    if (emergency) _finishMission(false, 'Superseded by $label');
    ++_manualGeneration;
    _stopControlLoop(); // Do not stream manual motor commands into autonomy.
    _voiceResetTimer?.cancel();
    throttle = pitch = roll = yaw = 0;
    _finishTelemetryRequest(
      false,
    ); // Next matching INFO must be requested after SET.
    final result = Completer<bool>();
    _missionResult = result;
    _missionExpected = expected;
    missionStatus = '$label sent; waiting for INFO';
    _missionTimer = Timer(const Duration(milliseconds: 1500), () {
      _finishMission(
        false,
        '$label rejected/unconfirmed: expected INFO state not observed',
      );
      _pushLog(missionStatus);
    });
    try {
      client.sendBytes(packet);
      _pushLog(missionStatus);
    } catch (error) {
      _finishMission(false, '$label send failed: $error');
      _pushLog(missionStatus);
    }
    return result.future;
  }

  Future<bool> requestState() => _requestTelemetry(
    DroneComms.comRequestPos,
    DroneComms.comReplyPos,
    'State',
  );
  Future<bool> requestStateEstimate() => _requestTelemetry(
    DroneComms.comRequestStateEst,
    DroneComms.comReplyStateEst,
    'State estimate',
  );
  Future<bool> requestInfo() => _requestTelemetry(
    DroneComms.comRequestInfo,
    DroneComms.comReplyInfo,
    'Info',
  );

  Future<bool> _requestTelemetry(int type, int reply, String label) {
    if (!isConnected || telemetryRequestPending) {
      _pushLog(
        '$label request unavailable: disconnected or another request is pending',
      );
      return Future.value(false);
    }
    final packet = _packetBuilder.headerOnly(
      messageType: type,
      toId: targetDroneId,
    );
    final result = Completer<bool>();
    _telemetryRequest = result;
    _requestId = packet[3];
    _requestDroneId = targetDroneId;
    _replyType = reply;
    _requestTimer = Timer(const Duration(seconds: 2), () {
      _finishTelemetryRequest(false);
      _pushLog('$label request timed out');
    });
    try {
      client.sendBytes(packet);
      if (type != DroneComms.comRequestInfo) {
        _pushLog('$label requested from $targetDroneCharacter');
      }
    } catch (error) {
      _finishTelemetryRequest(false);
      _pushLog('$label send failed: $error');
    }
    return result.future;
  }

  void _finishTelemetryRequest(bool success) {
    _requestTimer?.cancel();
    _requestTimer = null;
    final request = _telemetryRequest;
    _telemetryRequest = null;
    request?.complete(success);
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

  // This needs to be calibrated for the actual drone.
  static const double defaultHoverThrottle = 0.50;

  // Directional voice commands automatically center after this time.
  static const Duration voiceMovementDuration = Duration(milliseconds: 750);

  void _pushLog(String message) {
    log.insert(0, message);

    if (log.length > 300) {
      log.removeLast();
    }

    notifyListeners();
  }

  bool setTargetDroneId(String input) {
    if (isConnected || isConnecting) {
      _pushLog('Disconnect before changing the target drone ID');
      return false;
    }
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

    _stopMissionTracking();
    _finishTelemetryRequest(false);
    _replyBuffer.clear();
    telemetry = const DroneTelemetry();
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

    _replyBuffer.clear();
    telemetry = const DroneTelemetry();
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
      _infoPollTimer?.cancel();
      _infoPollTimer = Timer.periodic(
        const Duration(milliseconds: 500),
        (_) => _pollInfo(),
      );

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
    _stopMissionTracking();
    _finishTelemetryRequest(false);
    _replyBuffer.clear();
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
    _replyBuffer.addAll(bytes);
    while (_replyBuffer.length >= DroneComms.headerLen) {
      final size = switch (_replyBuffer[2]) {
        DroneComms.comPong ||
        DroneComms.comSuccess ||
        DroneComms.comFailure ||
        DroneComms.comAcknowledged ||
        DroneComms.comInvalid => 0,
        DroneComms.comReplyPos => 13,
        DroneComms.comReplyStateEst => 48,
        DroneComms.comReplyInfo => 20,
        0x24 || 0x25 || 0x26 || 0x27 => 0,
        DroneComms.comReplyWifi => 20,
        0x06 => 16, // Kent COM_REPLY_MOTORS
        _ => null,
      };
      if (size == null) {
        _replyBuffer.clear();
        _finishTelemetryRequest(false);
        _pushLog(
          'Unsupported reply type; cannot determine TCP message boundary',
        );
        return;
      }
      final length = DroneComms.headerLen + size;
      if (_replyBuffer.length < length) return;
      final packet = DronePacketBuilder.tryParse(
        Uint8List.fromList(_replyBuffer.sublist(0, length)),
      )!;
      _replyBuffer.removeRange(0, length);
      if (packet.toId != appDeviceId || packet.fromId != targetDroneId) {
        continue;
      }
      final matches =
          _telemetryRequest != null &&
          packet.messageId == _requestId &&
          packet.fromId == _requestDroneId;
      if (matches &&
          (packet.messageType == DroneComms.comFailure ||
              packet.messageType == DroneComms.comInvalid)) {
        _finishTelemetryRequest(false);
        _pushLog('Telemetry request rejected by drone');
        continue;
      }
      final update = switch (packet.messageType) {
        DroneComms.comReplyPos => DroneTelemetry.fromStateReply(packet.payload),
        DroneComms.comReplyStateEst => DroneTelemetry.fromStateEstimateReply(
          packet.payload,
        ),
        DroneComms.comReplyInfo => DroneTelemetry.fromInfoReply(packet.payload),
        _ => null,
      };
      if (update != null) {
        telemetry = telemetry.mergedWith(update);
        if (packet.messageType == DroneComms.comReplyInfo &&
            telemetry.autonomyMode != 0) {
          ++_manualGeneration;
          _stopControlLoop();
        }
        if (packet.messageType == DroneComms.comReplyInfo &&
            matches &&
            packet.messageType == _replyType) {
          lastInfoAt = DateTime.now();
          if (_missionExpected?.call(telemetry) == true) {
            _finishMission(
              true,
              'Requested mission state observed in INFO (not proof of execution)',
            );
            _pushLog(missionStatus);
          }
        }
        if (matches && packet.messageType == _replyType) {
          _finishTelemetryRequest(true);
        }
        _pushLog('Telemetry: $telemetry');
      }
    }
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

  // Firmware's com_step() (communication.cpp) reads all currently-available
  // TCP bytes into one buffer and processes only the first packet in it, with
  // no leftover-byte carryover - two sendBytes() calls with no gap can land
  // in the same read and silently drop the second packet. This delay is a
  // stopgap to keep the two sends in separate TCP reads; the real fix is
  // proper per-packet framing in wifiReceiving()/com_step() on the firmware
  // side.
  static const _packetGap = Duration(milliseconds: 60);

  Future<void> arm() async {
    if (missionCommandPending ||
        !missionInfoFresh ||
        telemetry.autonomyMode != 0) {
      _pushLog('Manual ARM blocked: select MANUAL and refresh INFO first');
      return;
    }
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
    final generation = ++_manualGeneration;
    final modePacket = _packetBuilder.flightMode(
      cmdMode: DroneComms.cmdModeNull,
      mode: DroneComms.flightModeActuation,
      toId: _targetDroneId,
    );
    client.sendBytes(modePacket);
    await Future.delayed(_packetGap);
    if (generation != _manualGeneration || !isConnected || _killed) return;

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
    final generation = ++_manualGeneration;
    _finishMission(false, 'Manual DISARM requested');
    _voiceResetTimer?.cancel();
    _voiceResetTimer = null;

    throttle = 0.0;
    pitch = 0.0;
    roll = 0.0;
    yaw = 0.0;

    _stopControlLoop();

    if (client.isConnected) {
      client.sendBytes(
        _packetBuilder.actuation(armed: false, toId: _targetDroneId),
      );
      await Future.delayed(_packetGap);
      if (generation != _manualGeneration || !isConnected || _killed) return;
      // Also return to NULL_MODE so flight_step() actively re-zeroes
      // cmd.motors every tick as a second line of defense, not just whatever
      // relied on the actuation flag alone.
      client.sendBytes(
        _packetBuilder.flightMode(
          cmdMode: DroneComms.cmdModeNull,
          mode: DroneComms.flightModeNull,
          toId: _targetDroneId,
        ),
      );
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

    ++_manualGeneration;
    _finishMission(false, 'KILL sent; onboard state unconfirmed');
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
    if (missionCommandPending ||
        !missionInfoFresh ||
        telemetry.autonomyMode != 0) {
      _pushLog('Manual loop blocked outside confirmed MANUAL mode');
      return;
    }
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

    _voiceResetTimer?.cancel();
    _voiceResetTimer = null;

    throttle = defaultHoverThrottle;
    pitch = 0.0;
    roll = 0.0;
    yaw = 0.0;

    _pushLog('Voice: HOVER — throttle=${throttle.toStringAsFixed(2)}');

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
        shapedThrottle + shapedRoll - shapedPitch - shapedYaw,
      );

      final motor1 = _clampMotor(
        shapedThrottle + shapedRoll + shapedPitch + shapedYaw,
      );

      final motor2 = _clampMotor(
        shapedThrottle - shapedRoll - shapedPitch + shapedYaw,
      );

      final motor3 = _clampMotor(
        shapedThrottle - shapedRoll + shapedPitch - shapedYaw,
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
    });
  }

  void _stopControlLoop() {
    _txTimer?.cancel();
    _txTimer = null;
  }

  @override
  void dispose() {
    _stopMissionTracking();
    _finishTelemetryRequest(false);
    _voiceResetTimer?.cancel();
    _stopControlLoop();
    _sub?.cancel();
    _packetSub?.cancel();

    // TcpClient is owned and disposed by Provider in main.dart.
    super.dispose();
  }
}
