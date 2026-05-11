import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../state/robot_state.dart';

class PidScreen extends StatefulWidget {
  const PidScreen({super.key});

  @override
  State<PidScreen> createState() => _PidScreenState();
}

class _PidScreenState extends State<PidScreen> {
  // Sliders are double-only here; SPEED is rounded when sent.
  double kp = 0.5, ki = 0.03, kd = 0.2;
  double mb = 2.0;
  double hkp = 2.0, hkd = 0.3, wg = 1.5;
  double lfac = 1.0, rfac = 1.0;
  double speed = 42;

  bool _seeded = false;

  void _seedFromTelemetry(RobotState s) {
    final t = s.telemetry;
    if (t.kp != null) kp = t.kp!;
    if (t.ki != null) ki = t.ki!;
    if (t.kd != null) kd = t.kd!;
    if (t.mb != null) mb = t.mb!;
    if (t.hkp != null) hkp = t.hkp!;
    if (t.hkd != null) hkd = t.hkd!;
    if (t.wg != null) wg = t.wg!;
    if (t.lfac != null) lfac = t.lfac!;
    if (t.rfac != null) rfac = t.rfac!;
    if (t.spd != null) speed = t.spd!.toDouble();
  }

  @override
  Widget build(BuildContext context) {
    final state = context.watch<RobotState>();
    if (!_seeded && state.telemetry.kp != null) {
      _seedFromTelemetry(state);
      _seeded = true;
    }
    final connected = state.connected;

    return SafeArea(
      child: ListView(
        padding: const EdgeInsets.all(12),
        children: [
          _section('Corridor PID', [
            _slider('Kp', kp, -5, 5, (v) => setState(() => kp = v)),
            _slider('Ki', ki, -1, 1, (v) => setState(() => ki = v)),
            _slider('Kd', kd, -3, 3, (v) => setState(() => kd = v)),
            _sendBtn(
                'Send PID',
                connected
                    ? () => state.send(
                        'PID=${kp.toStringAsFixed(3)},${ki.toStringAsFixed(3)},${kd.toStringAsFixed(3)}')
                    : null),
          ]),
          _section('Dead-band Margin', [
            _slider('MB (cm)', mb, 0, 10, (v) => setState(() => mb = v)),
            _sendBtn(
                'Send MB',
                connected
                    ? () => state.send('MB=${mb.toStringAsFixed(2)}')
                    : null),
          ]),
          _section('Heading Hold (PD)', [
            _slider('HKP', hkp, 0, 10, (v) => setState(() => hkp = v)),
            _slider('HKD', hkd, 0, 5, (v) => setState(() => hkd = v)),
            _slider('WG (wall→heading)', wg, 0, 5,
                (v) => setState(() => wg = v)),
            Row(
              children: [
                Expanded(
                  child: _sendBtn(
                      'Send HKP',
                      connected
                          ? () => state.send('HKP=${hkp.toStringAsFixed(3)}')
                          : null),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: _sendBtn(
                      'Send HKD',
                      connected
                          ? () => state.send('HKD=${hkd.toStringAsFixed(3)}')
                          : null),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: _sendBtn(
                      'Send WG',
                      connected
                          ? () => state.send('WG=${wg.toStringAsFixed(3)}')
                          : null),
                ),
              ],
            ),
          ]),
          _section('Motor', [
            _slider('Speed', speed, 0, 100,
                (v) => setState(() => speed = v),
                divisions: 100),
            _sendBtn(
                'Send Speed',
                connected
                    ? () => state.send('SPEED=${speed.round()}')
                    : null),
            _slider('LFAC', lfac, 0, 2, (v) => setState(() => lfac = v)),
            _slider('RFAC', rfac, 0, 2, (v) => setState(() => rfac = v)),
            Row(
              children: [
                Expanded(
                  child: _sendBtn(
                      'Send LFAC',
                      connected
                          ? () => state.send('LFAC=${lfac.toStringAsFixed(3)}')
                          : null),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: _sendBtn(
                      'Send RFAC',
                      connected
                          ? () => state.send('RFAC=${rfac.toStringAsFixed(3)}')
                          : null),
                ),
              ],
            ),
          ]),
          const SizedBox(height: 8),
          OutlinedButton.icon(
            icon: const Icon(Icons.refresh),
            label: const Text('Pull current values (GET)'),
            onPressed: connected
                ? () async {
                    await state.send('GET');
                    // RobotState updates telemetry asynchronously when the ack
                    // arrives; reseed on the next build via the _seeded gate.
                    setState(() => _seeded = false);
                  }
                : null,
          ),
        ],
      ),
    );
  }

  Widget _section(String title, List<Widget> children) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Text(title,
                style: const TextStyle(
                    fontWeight: FontWeight.w600, fontSize: 16)),
            const SizedBox(height: 6),
            ...children,
          ],
        ),
      ),
    );
  }

  Widget _slider(String label, double value, double min, double max,
      ValueChanged<double> onChanged,
      {int? divisions}) {
    return Row(
      children: [
        SizedBox(
          width: 88,
          child: Text(label, style: const TextStyle(fontSize: 13)),
        ),
        Expanded(
          child: Slider(
            value: value.clamp(min, max),
            min: min,
            max: max,
            divisions: divisions,
            label: value.toStringAsFixed(2),
            onChanged: onChanged,
          ),
        ),
        SizedBox(
          width: 56,
          child: Text(value.toStringAsFixed(2),
              textAlign: TextAlign.right,
              style: const TextStyle(fontFeatures: [
                FontFeature.tabularFigures(),
              ])),
        ),
      ],
    );
  }

  Widget _sendBtn(String label, VoidCallback? onPressed) {
    return Padding(
      padding: const EdgeInsets.only(top: 4),
      child: FilledButton(onPressed: onPressed, child: Text(label)),
    );
  }
}
