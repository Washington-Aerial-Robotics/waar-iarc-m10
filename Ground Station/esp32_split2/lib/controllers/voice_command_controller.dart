import 'package:flutter/foundation.dart';
import 'package:speech_to_text/speech_recognition_error.dart';
import 'package:speech_to_text/speech_recognition_result.dart';
import 'package:speech_to_text/speech_to_text.dart';

import '../protocol/drone_protocol.dart';
import 'drone_controller.dart';
import 'voice_command_parser.dart';

class VoiceCommandController extends ChangeNotifier {
  VoiceCommandController(this.droneController);

  final DroneController droneController;
  final SpeechToText _speech = SpeechToText();

  bool _available = false;
  bool _initializing = false;
  bool _listening = false;

  String _recognizedText = '';
  String _lastCommand = '';
  String? _error;

  bool get available => _available;
  bool get initializing => _initializing;
  bool get listening => _listening;

  String get recognizedText => _recognizedText;
  String get lastCommand => _lastCommand;
  String? get error => _error;

  Future<void> initialize() async {
    if (_available || _initializing) {
      return;
    }

    _initializing = true;
    _error = null;
    notifyListeners();

    try {
      _available = await _speech.initialize(
        onStatus: _handleStatus,
        onError: _handleError,
        debugLogging: false,
      );

      if (!_available) {
        _error = 'Speech recognition is unavailable';
      }
    } catch (error) {
      _available = false;
      _error = 'Speech initialization failed: $error';
    } finally {
      _initializing = false;
      notifyListeners();
    }
  }

  Future<void> startListening() async {
    if (_listening) {
      return;
    }

    if (!_available) {
      await initialize();
    }

    if (!_available) {
      return;
    }

    _recognizedText = '';
    _lastCommand = '';
    _error = null;

    try {
      await _speech.listen(
        onResult: _handleResult,
        listenOptions: SpeechListenOptions(
          listenMode: ListenMode.confirmation,
          partialResults: true,
          cancelOnError: true,
          onDevice: false,
          autoPunctuation: false,
          listenFor: const Duration(seconds: 6),
          pauseFor: const Duration(seconds: 2),
        ),
      );

      _listening = _speech.isListening;
    } catch (error) {
      _error = 'Unable to start listening: $error';
      _listening = false;
    }

    notifyListeners();
  }

  Future<void> stopListening() async {
    try {
      await _speech.stop();
    } catch (error) {
      _error = 'Unable to stop listening: $error';
    }

    _listening = false;
    notifyListeners();
  }

  Future<void> cancelListening() async {
    try {
      await _speech.cancel();
    } catch (error) {
      _error = 'Unable to cancel listening: $error';
    }

    _listening = false;
    notifyListeners();
  }

  void _handleStatus(String status) {
    _listening = status == SpeechToText.listeningStatus;

    if (status == SpeechToText.doneStatus ||
        status == SpeechToText.notListeningStatus) {
      _listening = false;
    }

    notifyListeners();
  }

  void _handleError(SpeechRecognitionError error) {
    _error = error.errorMsg;
    _listening = false;

    notifyListeners();
  }

  void _handleResult(SpeechRecognitionResult result) {
    _recognizedText = result.recognizedWords
        .trim()
        .toLowerCase();

    notifyListeners();

    if (!result.finalResult) {
      return;
    }

    _executeCommand(_recognizedText);
  }

  void _executeCommand(String phrase) {
    final command = parseVoiceCommand(phrase);

    // Qualification commands are only acted on while Qualification mode is
    // actually selected, and manual commands are ignored while it is -
    // this is a second, independent safeguard on top of the parser's own
    // distinct phrase set (voice_command_parser.dart), and matches the
    // firmware's own behavior (commander_qualificationCommand() ignores
    // QUALCMD_* while autonomyMode != AUTONOMY_QUALIFICATION).
    final inQualificationMode = droneController.telemetry.autonomyMode ==
        DroneComms.autonomyQualification;

    if (command != null && isQualificationVoiceCommand(command)) {
      if (!inQualificationMode) {
        _lastCommand = 'ignored (not in Qualification mode): $phrase';
        notifyListeners();
        return;
      }
      _executeQualificationCommand(command);
      return;
    }

    if (inQualificationMode) {
      // Manual commands are meaningless (and unwired) while Qualification
      // is selected - the firmware's own state machine, not joystick
      // input, is what moves the drone in this mode.
      _lastCommand = 'ignored (Qualification mode active): $phrase';
      notifyListeners();
      return;
    }

    if (command == VoiceCommand.rotateLeft) {
      droneController.voiceRotateLeft();
      _lastCommand = 'rotate left';
    } else if (command == VoiceCommand.rotateRight) {
      droneController.voiceRotateRight();
      _lastCommand = 'rotate right';
    } else if (command == VoiceCommand.forward) {
      droneController.voiceForward();
      _lastCommand = 'forward';
    } else if (command == VoiceCommand.backward) {
      droneController.voiceBackward();
      _lastCommand = 'backward';
    } else if (command == VoiceCommand.moveLeft) {
      droneController.voiceMoveLeft();
      _lastCommand = 'move left';
    } else if (command == VoiceCommand.moveRight) {
      droneController.voiceMoveRight();
      _lastCommand = 'move right';
    } else if (command == VoiceCommand.hover) {
      droneController.voiceHover();
      _lastCommand = 'hover';
    } else if (command == VoiceCommand.stop) {
      droneController.voiceStop();
      _lastCommand = 'stop';
    } else if (command == VoiceCommand.up) {
      droneController.voiceUp();
      _lastCommand = 'up';
    } else if (command == VoiceCommand.down) {
      droneController.voiceDown();
      _lastCommand = 'down';
    } else {
      _lastCommand = 'unrecognized: $phrase';
    }

    notifyListeners();
  }

  void _executeQualificationCommand(VoiceCommand command) {
    switch (command) {
      case VoiceCommand.qualLaunch:
        droneController.sendQualCommand(DroneComms.qualCmdLaunch);
        _lastCommand = 'qualification launch';
        break;
      case VoiceCommand.qualBeginOrbit:
        droneController.sendQualCommand(DroneComms.qualCmdBeginOrbit);
        _lastCommand = 'begin orbit';
        break;
      case VoiceCommand.qualHold:
        droneController.sendQualCommand(DroneComms.qualCmdHold);
        _lastCommand = 'orbit hold';
        break;
      case VoiceCommand.qualLand:
        droneController.sendQualCommand(DroneComms.qualCmdLand);
        _lastCommand = 'qualification land';
        break;
      case VoiceCommand.qualAbort:
        droneController.sendQualCommand(DroneComms.qualCmdAbort);
        _lastCommand = 'abort qualification';
        break;
      default:
        break;
    }
    notifyListeners();
  }

  @override
  void dispose() {
    _speech.cancel();
    super.dispose();
  }
}
