import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'controllers/drone_controller.dart';
import 'controllers/voice_command_controller.dart';
import 'screens/drone_remote_control.dart';
import 'screens/drone_tcp_console.dart';
import 'screens/qualification_control.dart';
import 'services/tcp_client.dart';

void main() {
  runApp(
    MultiProvider(
      providers: [
        Provider<TcpClient>(
          create: (_) => TcpClient(),
          dispose: (_, client) => client.dispose(),
        ),
        ChangeNotifierProvider<DroneController>(
          create: (context) => DroneController(
            context.read<TcpClient>(),
          ),
        ),
        ChangeNotifierProvider<VoiceCommandController>(
          create: (context) {
            return VoiceCommandController(
              context.read<DroneController>(),
            )..initialize();
          },
        ),
      ],
      child: const MyApp(),
    ),
  );
}

class MyApp extends StatelessWidget {
  const MyApp({
    super.key,
  });

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      debugShowCheckedModeBanner: false,
      home: NavigationShell(),
    );
  }
}

class NavigationShell extends StatefulWidget {
  const NavigationShell({
    super.key,
  });

  @override
  State<NavigationShell> createState() {
    return _NavigationShellState();
  }
}

class _NavigationShellState
    extends State<NavigationShell> {
  int _index = 0;

  final List<Widget> _pages = const [
    DroneTcpConsole(),
    DroneRemoteControl(),
    QualificationControl(),
  ];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: IndexedStack(
        index: _index,
        children: _pages,
      ),
      bottomNavigationBar: NavigationBar(
        selectedIndex: _index,
        onDestinationSelected: (index) {
          setState(() {
            _index = index;
          });
        },
        destinations: const [
          NavigationDestination(
            icon: Icon(Icons.terminal),
            label: 'Console',
          ),
          NavigationDestination(
            icon: Icon(Icons.gamepad),
            label: 'Remote',
          ),
          NavigationDestination(
            icon: Icon(Icons.rule),
            label: 'Mode',
          ),
        ],
      ),
    );
  }
}