import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../state/robot_state.dart';

class DashboardScreen extends StatefulWidget {
  const DashboardScreen({super.key});

  @override
  State<DashboardScreen> createState() => _DashboardScreenState();
}

class _DashboardScreenState extends State<DashboardScreen> {
  late final TextEditingController _hostCtl;
  late final TextEditingController _portCtl;
  bool _seeded = false;

  @override
  void initState() {
    super.initState();
    _hostCtl = TextEditingController();
    _portCtl = TextEditingController(text: '1234');
  }

  @override
  void dispose() {
    _hostCtl.dispose();
    _portCtl.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final state = context.watch<RobotState>();
    if (!_seeded && (state.host.isNotEmpty || state.port != 0)) {
      _hostCtl.text = state.host;
      _portCtl.text = state.port.toString();
      _seeded = true;
    }

    final t = state.telemetry;
    return SafeArea(
      child: SingleChildScrollView(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            _connectionCard(context, state),
            const SizedBox(height: 12),
            Row(
              children: [
                Expanded(
                  child: _sensorTile('Left', t.left, Colors.blue),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: _sensorTile('Front', t.front, Colors.orange),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: _sensorTile('Right', t.right, Colors.green),
                ),
              ],
            ),
            const SizedBox(height: 12),
            _headingCard(t.angle, t.target),
            const SizedBox(height: 12),
            _modeCard(state),
            const SizedBox(height: 12),
            _runCard(state, t.running),
            const SizedBox(height: 12),
            _turnsCard(state),
          ],
        ),
      ),
    );
  }

  Widget _connectionCard(BuildContext context, RobotState state) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Row(
              children: [
                Icon(Icons.circle,
                    size: 14,
                    color: state.connected ? Colors.green : Colors.red),
                const SizedBox(width: 8),
                Text(state.connected ? 'Connected' : 'Disconnected',
                    style: Theme.of(context).textTheme.titleMedium),
              ],
            ),
            const SizedBox(height: 8),
            Row(
              children: [
                Expanded(
                  flex: 3,
                  child: TextField(
                    controller: _hostCtl,
                    keyboardType: TextInputType.text,
                    decoration: const InputDecoration(
                      labelText: 'Robot IP',
                      hintText: '192.168.1.42',
                      isDense: true,
                      border: OutlineInputBorder(),
                    ),
                  ),
                ),
                const SizedBox(width: 8),
                Expanded(
                  flex: 1,
                  child: TextField(
                    controller: _portCtl,
                    keyboardType: TextInputType.number,
                    decoration: const InputDecoration(
                      labelText: 'Port',
                      isDense: true,
                      border: OutlineInputBorder(),
                    ),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 8),
            FilledButton.icon(
              icon: Icon(state.connected ? Icons.link_off : Icons.link),
              label: Text(state.connected ? 'Disconnect' : 'Connect'),
              onPressed: () async {
                if (state.connected) {
                  await state.disconnect();
                } else {
                  final messenger = ScaffoldMessenger.of(context);
                  final port =
                      int.tryParse(_portCtl.text.trim()) ?? RobotState.defaultPort;
                  final ok = await state.connect(_hostCtl.text.trim(), port);
                  if (!ok) {
                    messenger.showSnackBar(
                      const SnackBar(content: Text('Connection failed')),
                    );
                  }
                }
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _sensorTile(String label, double? cm, Color color) {
    return Card(
      color: color.withValues(alpha: 0.10),
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 14, horizontal: 8),
        child: Column(
          children: [
            Text(label, style: TextStyle(color: color, fontWeight: FontWeight.w600)),
            const SizedBox(height: 6),
            Text(
              cm == null ? '---' : cm.toStringAsFixed(1),
              style: TextStyle(
                  fontSize: 26, fontWeight: FontWeight.bold, color: color),
            ),
            const Text('cm', style: TextStyle(fontSize: 11, color: Colors.grey)),
          ],
        ),
      ),
    );
  }

  Widget _headingCard(double? angle, double? target) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Row(
          children: [
            SizedBox(
              width: 96,
              height: 96,
              child: CustomPaint(
                painter: _CompassPainter(angle ?? 0, target),
              ),
            ),
            const SizedBox(width: 16),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text('IMU Heading',
                      style: TextStyle(fontWeight: FontWeight.w600)),
                  const SizedBox(height: 6),
                  Text(angle == null ? '--- °' : '${angle.toStringAsFixed(1)} °',
                      style: const TextStyle(
                          fontSize: 28, fontWeight: FontWeight.bold)),
                  Text(target == null
                      ? 'target ---'
                      : 'target ${target.toStringAsFixed(1)} °',
                      style: const TextStyle(color: Colors.grey)),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _modeCard(RobotState state) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Row(
          children: [
            const Icon(Icons.swap_horiz),
            const SizedBox(width: 8),
            const Text('Mode', style: TextStyle(fontWeight: FontWeight.w600)),
            const Spacer(),
            SegmentedButton<DriveMode>(
              segments: const [
                ButtonSegment(value: DriveMode.auto, label: Text('AUTO')),
                ButtonSegment(value: DriveMode.manual, label: Text('MANUAL')),
              ],
              selected: {state.mode},
              onSelectionChanged: state.connected
                  ? (s) => state.setMode(s.first)
                  : null,
            ),
          ],
        ),
      ),
    );
  }

  Widget _runCard(RobotState state, int? running) {
    final isRunning = running == 1;

    String stateName = 'UNKNOWN';
    switch (state.telemetry.state) {
      case 0: stateName = 'IDLE'; break;
      case 1: stateName = 'FORWARD'; break;
      case 2: stateName = 'TURN_LEFT'; break;
      case 3: stateName = 'TURN_RIGHT'; break;
      case 4: stateName = 'DEAD_END'; break;
    }

    final latestEvent = state.fsmEvents.isNotEmpty ? state.fsmEvents.last : null;

    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          children: [
            Row(
              children: [
                Icon(isRunning ? Icons.play_circle : Icons.pause_circle,
                    color: isRunning ? Colors.green : Colors.red, size: 28),
                const SizedBox(width: 8),
                Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(isRunning ? 'RUNNING ($stateName)' : 'STOPPED ($stateName)',
                        style: const TextStyle(fontWeight: FontWeight.bold)),
                    if (latestEvent != null)
                      Text('Reason: ${latestEvent.reason}',
                          style: const TextStyle(fontSize: 11, color: Colors.grey)),
                  ],
                ),
                const Spacer(),
                OutlinedButton(
                  onPressed: state.connected ? () => state.send('STOP') : null,
                  child: const Text('STOP'),
                ),
                const SizedBox(width: 8),
                FilledButton(
                  onPressed: state.connected ? () => state.send('GO') : null,
                  child: const Text('GO'),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _turnsCard(RobotState state) {
    final turns = state.turns;
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Row(
              children: [
                const Icon(Icons.turn_right_outlined),
                const SizedBox(width: 8),
                Text('Turns: ${turns.length}',
                    style: const TextStyle(fontWeight: FontWeight.w600)),
                const Spacer(),
                IconButton(
                  icon: const Icon(Icons.delete_outline, size: 20),
                  tooltip: 'Clear',
                  onPressed: turns.isEmpty ? null : state.clearTurns,
                ),
              ],
            ),
            if (turns.isNotEmpty) ...[
              const SizedBox(height: 8),
              Wrap(
                spacing: 6,
                runSpacing: 6,
                children: turns.map((t) {
                  final isLeft = t == 'L';
                  return Chip(
                    label: Text(t,
                        style: const TextStyle(
                            fontWeight: FontWeight.bold, fontSize: 13)),
                    backgroundColor: isLeft
                        ? Colors.blue.withValues(alpha: 0.15)
                        : Colors.green.withValues(alpha: 0.15),
                    side: BorderSide(
                        color: isLeft ? Colors.blue : Colors.green, width: 1),
                    padding: EdgeInsets.zero,
                    labelPadding:
                        const EdgeInsets.symmetric(horizontal: 8, vertical: 0),
                  );
                }).toList(),
              ),
            ],
          ],
        ),
      ),
    );
  }
}

class _CompassPainter extends CustomPainter {
  final double angleDeg;
  final double? targetDeg;
  _CompassPainter(this.angleDeg, this.targetDeg);

  @override
  void paint(Canvas canvas, Size size) {
    final c = size.center(Offset.zero);
    final r = size.shortestSide / 2 - 4;

    final ring = Paint()
      ..color = Colors.grey.shade400
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;
    canvas.drawCircle(c, r, ring);

    void tick(double deg, double len, Color col) {
      final rad = (deg - 90) * math.pi / 180.0;
      final p1 = Offset(c.dx + math.cos(rad) * (r - len),
          c.dy + math.sin(rad) * (r - len));
      final p2 = Offset(c.dx + math.cos(rad) * r, c.dy + math.sin(rad) * r);
      canvas.drawLine(
          p1,
          p2,
          Paint()
            ..color = col
            ..strokeWidth = 2);
    }

    for (int i = 0; i < 360; i += 30) {
      tick(i.toDouble(), i % 90 == 0 ? 10 : 5, Colors.grey.shade500);
    }

    if (targetDeg != null) {
      final rad = (targetDeg! - 90) * math.pi / 180.0;
      canvas.drawLine(
        c,
        Offset(c.dx + math.cos(rad) * r, c.dy + math.sin(rad) * r),
        Paint()
          ..color = Colors.orange
          ..strokeWidth = 2,
      );
    }

    final rad = (angleDeg - 90) * math.pi / 180.0;
    final tip = Offset(c.dx + math.cos(rad) * (r - 6),
        c.dy + math.sin(rad) * (r - 6));
    canvas.drawLine(
      c,
      tip,
      Paint()
        ..color = Colors.blue
        ..strokeWidth = 4
        ..strokeCap = StrokeCap.round,
    );
    canvas.drawCircle(c, 3, Paint()..color = Colors.blue);
  }

  @override
  bool shouldRepaint(covariant _CompassPainter old) =>
      old.angleDeg != angleDeg || old.targetDeg != targetDeg;
}
