enum VoiceCommand {
  rotateLeft,
  rotateRight,
  forward,
  backward,
  moveLeft,
  moveRight,
  hover,
  stop,
  up,
  down,
}

/// Parses only well-defined command phrases. This deliberately avoids fuzzy
/// matching: guessing is unsafe when the result controls a flying vehicle.
VoiceCommand? parseVoiceCommand(String transcript) {
  var phrase = transcript
      .toLowerCase()
      .replaceAll(RegExp(r'[^a-z0-9\s]'), ' ')
      .replaceAll(RegExp(r'\s+'), ' ')
      .trim();

  // Speech recognizers commonly include conversational padding around the
  // actual short command.
  phrase = phrase
      .replaceFirst(RegExp(r'^hey\s+'), '')
      .replaceFirst(RegExp(r'^drone\s+'), '')
      .replaceFirst(RegExp(r'^(?:please|can you|could you|would you)\s+'), '')
      .replaceAll(RegExp(r'(?: please| now)$'), '')
      .trim();

  const phrases = <String, VoiceCommand>{
    'rotate left': VoiceCommand.rotateLeft,
    'turn left': VoiceCommand.rotateLeft,
    'yaw left': VoiceCommand.rotateLeft,
    'rotate right': VoiceCommand.rotateRight,
    'rotate write': VoiceCommand.rotateRight,
    'turn right': VoiceCommand.rotateRight,
    'turn write': VoiceCommand.rotateRight,
    'yaw right': VoiceCommand.rotateRight,
    'yaw write': VoiceCommand.rotateRight,
    'move forward': VoiceCommand.forward,
    'move forwards': VoiceCommand.forward,
    'go forward': VoiceCommand.forward,
    'go forwards': VoiceCommand.forward,
    'forward': VoiceCommand.forward,
    'forwards': VoiceCommand.forward,
    'move backward': VoiceCommand.backward,
    'move backwards': VoiceCommand.backward,
    'go backward': VoiceCommand.backward,
    'go backwards': VoiceCommand.backward,
    'backward': VoiceCommand.backward,
    'backwards': VoiceCommand.backward,
    'back': VoiceCommand.backward,
    'move left': VoiceCommand.moveLeft,
    'move to the left': VoiceCommand.moveLeft,
    'go left': VoiceCommand.moveLeft,
    'go to the left': VoiceCommand.moveLeft,
    'slide left': VoiceCommand.moveLeft,
    'strafe left': VoiceCommand.moveLeft,
    'move right': VoiceCommand.moveRight,
    'move write': VoiceCommand.moveRight,
    'move to the right': VoiceCommand.moveRight,
    'move to the write': VoiceCommand.moveRight,
    'go right': VoiceCommand.moveRight,
    'go write': VoiceCommand.moveRight,
    'go to the right': VoiceCommand.moveRight,
    'go to the write': VoiceCommand.moveRight,
    'slide right': VoiceCommand.moveRight,
    'slide write': VoiceCommand.moveRight,
    'strafe right': VoiceCommand.moveRight,
    'strafe write': VoiceCommand.moveRight,
    'hover': VoiceCommand.hover,
    'maintain altitude': VoiceCommand.hover,
    'hold altitude': VoiceCommand.hover,
    'stay still': VoiceCommand.hover,
    'stop': VoiceCommand.stop,
    'hold': VoiceCommand.stop,
    'center controls': VoiceCommand.stop,
    'stop moving': VoiceCommand.stop,
    'go up': VoiceCommand.up,
    'move up': VoiceCommand.up,
    'ascend': VoiceCommand.up,
    'increase altitude': VoiceCommand.up,
    'up': VoiceCommand.up,
    'go down': VoiceCommand.down,
    'move down': VoiceCommand.down,
    'descend': VoiceCommand.down,
    'decrease altitude': VoiceCommand.down,
    'down': VoiceCommand.down,
  };

  return phrases[phrase];
}
