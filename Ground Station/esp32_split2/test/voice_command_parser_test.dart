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

  test('recognizes the qualification command vocabulary', () {
    expect(parseVoiceCommand('Qualification launch'), VoiceCommand.qualLaunch);
    expect(parseVoiceCommand('begin orbit'), VoiceCommand.qualBeginOrbit);
    expect(parseVoiceCommand('start orbit'), VoiceCommand.qualBeginOrbit);
    expect(parseVoiceCommand('orbit hold'), VoiceCommand.qualHold);
    expect(parseVoiceCommand('Qualification land!'), VoiceCommand.qualLand);
    expect(parseVoiceCommand('abort qualification'), VoiceCommand.qualAbort);
  });

  test('qualification phrases never collide with manual ones', () {
    // Bare "hold" must keep meaning the manual stop command, not the
    // qualification orbit-hold command - they are deliberately different
    // phrases so mode-based routing is never the only thing preventing
    // ambiguity.
    expect(parseVoiceCommand('hold'), VoiceCommand.stop);
    expect(isQualificationVoiceCommand(VoiceCommand.stop), isFalse);
    expect(isQualificationVoiceCommand(VoiceCommand.qualHold), isTrue);
  });
}
