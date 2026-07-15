import 'package:flutter/foundation.dart';
import 'package:speech_to_text/speech_recognition_error.dart';
import 'package:speech_to_text/speech_recognition_result.dart';
import 'package:speech_to_text/speech_to_text.dart';

import 'drone_controller.dart';

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
    final normalized = phrase
        .toLowerCase()
        .replaceAll(
          RegExp(r'[^a-z0-9\s]'),
          ' ',
        )
        .replaceAll(
          RegExp(r'\s+'),
          ' ',
        )
        .trim();

    if (_matches(
      normalized,
      const [
        'rotate left',
        'turn left',
        'yaw left',
      ],
    )) {
      droneController.voiceRotateLeft();
      _lastCommand = 'rotate left';
    } else if (_matches(
      normalized,
      const [
        'rotate right',
        'turn right',
        'yaw right',
      ],
    )) {
      droneController.voiceRotateRight();
      _lastCommand = 'rotate right';
    } else if (_matches(
      normalized,
      const [
        'move forward',
        'go forward',
        'forward',
      ],
    )) {
      droneController.voiceForward();
      _lastCommand = 'forward';
    } else if (_matches(
      normalized,
      const [
        'move backward',
        'go backward',
        'backward',
        'back',
      ],
    )) {
      droneController.voiceBackward();
      _lastCommand = 'backward';
    } else if (_matches(
      normalized,
      const [
        'move left',
        'slide left',
        'strafe left',
      ],
    )) {
      droneController.voiceMoveLeft();
      _lastCommand = 'move left';
    } else if (_matches(
      normalized,
      const [
        'move right',
        'slide right',
        'strafe right',
      ],
    )) {
      droneController.voiceMoveRight();
      _lastCommand = 'move right';
    } else if (_matches(
      normalized,
      const [
        'hover',
        'maintain altitude',
        'hold altitude',
      ],
    )) {
      droneController.voiceHover();
      _lastCommand = 'hover';
    } else if (_matches(
      normalized,
      const [
        'stop',
        'hold',
        'center controls',
        'stop moving',
      ],
    )) {
      droneController.voiceStop();
      _lastCommand = 'stop';
    } else if (_matches(
      normalized,
      const [
        'go up',
        'move up',
        'ascend',
        'up',
      ],
    )) {
      droneController.voiceUp();
      _lastCommand = 'up';
    } else if (_matches(
      normalized,
      const [
        'go down',
        'move down',
        'descend',
        'down',
      ],
    )) {
      droneController.voiceDown();
      _lastCommand = 'down';
    } else {
      _lastCommand = 'unrecognized: $normalized';
    }

    notifyListeners();
  }

  bool _matches(
    String recognizedPhrase,
    List<String> acceptedPhrases,
  ) {
    return acceptedPhrases.any(
      (command) => recognizedPhrase == command,
    );
  }

  @override
  void dispose() {
    _speech.cancel();
    super.dispose();
  }
}