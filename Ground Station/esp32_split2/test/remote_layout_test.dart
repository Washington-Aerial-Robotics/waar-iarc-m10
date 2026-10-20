import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:esp32_split/controllers/drone_controller.dart';
import 'package:esp32_split/controllers/voice_command_controller.dart';
import 'package:esp32_split/screens/drone_remote_control.dart';
import 'package:esp32_split/services/tcp_client.dart';

void main() {
  for (final size in [const Size(844, 390), const Size(667, 375)]) {
    testWidgets('landscape controls fit $size with navigation and safe areas', (
      tester,
    ) async {
      tester.view.physicalSize = size;
      tester.view.devicePixelRatio = 1;
      addTearDown(tester.view.resetPhysicalSize);
      addTearDown(tester.view.resetDevicePixelRatio);
      final client = TcpClient();
      final drone = DroneController(client);
      final voice = VoiceCommandController(drone);
      await tester.pumpWidget(
        MultiProvider(
          providers: [
            ChangeNotifierProvider.value(value: drone),
            ChangeNotifierProvider.value(value: voice),
          ],
          child: MaterialApp(
            home: MediaQuery(
              data: MediaQueryData(
                size: size,
                padding: const EdgeInsets.fromLTRB(44, 0, 44, 21),
              ),
              child: Scaffold(
                body: const DroneRemoteControl(),
                bottomNavigationBar: NavigationBar(
                  destinations: const [
                    NavigationDestination(
                      icon: Icon(Icons.terminal),
                      label: 'Console',
                    ),
                    NavigationDestination(
                      icon: Icon(Icons.gamepad),
                      label: 'Remote',
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      );
      expect(tester.takeException(), isNull);
      expect(find.text('KILL').hitTestable(), findsOneWidget);
      expect(find.text('Throttle / Yaw'), findsOneWidget);
      expect(find.text('Pitch / Roll'), findsOneWidget);
      await tester.pumpWidget(const SizedBox());
      drone.dispose();
      client.dispose();
    });
  }
}
