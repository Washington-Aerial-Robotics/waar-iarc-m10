import 'package:esp32_split/controllers/voice_command_parser.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('accepts natural command wording and punctuation', () {
    expect(parseVoiceCommand('Please move forwards!'), VoiceCommand.forward);
    expect(parseVoiceCommand('Drone, go to the left now'), VoiceCommand.moveLeft);
    expect(parseVoiceCommand('Could you increase altitude?'), VoiceCommand.up);
  });

  test('handles common right/write speech-recognition substitution', () {
    expect(parseVoiceCommand('turn write'), VoiceCommand.rotateRight);
    expect(parseVoiceCommand('move to the write'), VoiceCommand.moveRight);
  });

  test('keeps ambiguous or unrelated speech from controlling the drone', () {
    expect(parseVoiceCommand('left'), isNull);
    expect(parseVoiceCommand('write'), isNull);
    expect(parseVoiceCommand('move'), isNull);
    expect(parseVoiceCommand('fly into the building'), isNull);
  });
}
