import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:esp32_split/controllers/drone_controller.dart';
import 'package:esp32_split/widgets/mission_panel.dart';
import 'mission_commands_test.dart' show info;
import 'telemetry_requests_test.dart' show TestClient;

void main() {
  testWidgets(
    'mission panel shows INFO and keeps emergency actions available',
    (tester) async {
      final client = TestClient();
      final controller = DroneController(client);
      await controller.connect('test', 70);
      final request = controller.requestInfo();
      client.reply(0x03, info(armed: true, mode: 1, sensors: 0));
      await request;
      await tester.pumpWidget(
        ChangeNotifierProvider.value(
          value: controller,
          child: const MaterialApp(home: Scaffold(body: MissionPanel())),
        ),
      );
      expect(tester.takeException(), isNull);
      expect(
        tester
            .widget<ElevatedButton>(
              find.widgetWithText(ElevatedButton, 'LAUNCH'),
            )
            .onPressed,
        isNull,
      );
      expect(
        tester
            .widget<ElevatedButton>(
              find.widgetWithText(ElevatedButton, 'QUAL LAND'),
            )
            .onPressed,
        isNotNull,
      );
      expect(
        tester
            .widget<ElevatedButton>(
              find.widgetWithText(ElevatedButton, 'SQUARE ABORT'),
            )
            .onPressed,
        isNotNull,
      );
      expect(
        tester
            .widget<ElevatedButton>(find.widgetWithText(ElevatedButton, 'KILL'))
            .onPressed,
        isNotNull,
      );
      expect(
        tester
            .widget<DropdownButtonFormField<int>>(
              find.byType(DropdownButtonFormField<int>).first,
            )
            .onChanged,
        isNull,
      );
      await tester.pumpWidget(const SizedBox());
      controller.dispose();
      await client.incoming.close();
      client.dispose();
    },
  );
}
