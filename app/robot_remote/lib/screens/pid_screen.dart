import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../state/robot_state.dart';

class PidScreen extends StatefulWidget {
  const PidScreen({super.key});

  @override
  State<PidScreen> createState() => _PidScreenState();
}

class _PidScreenState extends State<PidScreen> {
  // Heading hold
  double hkp = 2.0, hkd = 0.3, wg = 1.5, wgd = 0.08;
  // Dead-band & speed
  double mb = 2.0, speed = 60, speedFloor = 35;
  // Motor factors
  double lfac = 1.0, rfac = 1.0;
  // EMA & sensitivity
  double ema = 0.45, sensitivity = 20.0;
  // Front obstacle thresholds
  double frontDecel = 35.0, frontStop = 20.0;
  // Wall detection
  double dFar = 30.0, openWall = 20.0, desiredWall = 15.0;
  // Turn PD
  double turnKp = 7.0, turnKd = 0.7;
  double turnStopDeg = 2.5, turnAngleDeg = 90.0;
  double turnTimeoutMs = 6000;
  // Turn speed limits
  double turnMaxSpeed = 70, turnMinSpeed = 35;
  // Turn approach zone
  double turnApproachDeg = 15.0;
  double turnApproachMax = 32, turnApproachMin = 28;

  bool _seeded = false;

  void _seedFromTelemetry(RobotState s) {
    final t = s.telemetry;
    if (t.hkp != null) hkp = t.hkp!;
    if (t.hkd != null) hkd = t.hkd!;
    if (t.wg != null) wg = t.wg!;
    if (t.wgd != null) wgd = t.wgd!;
    if (t.mb != null) mb = t.mb!;
    if (t.spd != null) speed = t.spd!.toDouble();
    if (t.speedFloor != null) speedFloor = t.speedFloor!.toDouble();
    if (t.lfac != null) lfac = t.lfac!;
    if (t.rfac != null) rfac = t.rfac!;
    if (t.ema != null) ema = t.ema!;
    if (t.sensitivity != null) sensitivity = t.sensitivity!;
    if (t.frontDecel != null) frontDecel = t.frontDecel!;
    if (t.frontStop != null) frontStop = t.frontStop!;
    if (t.dFar != null) dFar = t.dFar!;
    if (t.openWall != null) openWall = t.openWall!;
    if (t.desiredWall != null) desiredWall = t.desiredWall!;
    if (t.turnKp != null) turnKp = t.turnKp!;
    if (t.turnKd != null) turnKd = t.turnKd!;
    if (t.turnStopDeg != null) turnStopDeg = t.turnStopDeg!;
    if (t.turnAngleDeg != null) turnAngleDeg = t.turnAngleDeg!;
    if (t.turnTimeoutMs != null) turnTimeoutMs = t.turnTimeoutMs!.toDouble();
    if (t.turnMaxSpeed != null) turnMaxSpeed = t.turnMaxSpeed!.toDouble();
    if (t.turnMinSpeed != null) turnMinSpeed = t.turnMinSpeed!.toDouble();
    if (t.turnApproachDeg != null) turnApproachDeg = t.turnApproachDeg!;
    if (t.turnApproachMax != null) turnApproachMax = t.turnApproachMax!.toDouble();
    if (t.turnApproachMin != null) turnApproachMin = t.turnApproachMin!.toDouble();
  }

  @override
  Widget build(BuildContext context) {
    final state = context.watch<RobotState>();
    if (!_seeded && state.telemetry.hkp != null) {
      _seedFromTelemetry(state);
      _seeded = true;
    }
    final c = state.connected;

    return SafeArea(
      child: ListView(
        padding: const EdgeInsets.all(12),
        children: [
          // ── Heading Hold ──────────────────────────────────────────
          _section('Heading Hold PD', [
            _slider('HKP', hkp, 0, 10, (v) => setState(() => hkp = v)),
            _slider('HKD', hkd, 0, 5, (v) => setState(() => hkd = v)),
            _slider('WG (wall→hdg)', wg, 0, 5, (v) => setState(() => wg = v)),
            _slider('WGD (wall d/dt)', wgd, 0, 1,
                (v) => setState(() => wgd = v)),
            Row(children: [
              Expanded(child: _sendBtn('Send HKP',
                  c ? () => state.send('HKP=${hkp.toStringAsFixed(3)}') : null)),
              const SizedBox(width: 8),
              Expanded(child: _sendBtn('Send HKD',
                  c ? () => state.send('HKD=${hkd.toStringAsFixed(3)}') : null)),
            ]),
            Row(children: [
              Expanded(child: _sendBtn('Send WG',
                  c ? () => state.send('WG=${wg.toStringAsFixed(3)}') : null)),
              const SizedBox(width: 8),
              Expanded(child: _sendBtn('Send WGD',
                  c ? () => state.send('WGD=${wgd.toStringAsFixed(4)}') : null)),
            ]),
          ]),

          // ── Motor ─────────────────────────────────────────────────
          _section('Motor', [
            _slider('Speed (%)', speed, 0, 100,
                (v) => setState(() => speed = v), divisions: 100),
            _sendBtn('Send Speed',
                c ? () => state.send('SPEED=${speed.round()}') : null),
            _slider('Speed Floor (%)', speedFloor, 0, 100,
                (v) => setState(() => speedFloor = v), divisions: 100),
            _sendBtn('Send Speed Floor',
                c ? () => state.send('SFLOOR=${speedFloor.round()}') : null),
            _slider('LFAC', lfac, 0, 2, (v) => setState(() => lfac = v)),
            _slider('RFAC', rfac, 0, 2, (v) => setState(() => rfac = v)),
            Row(children: [
              Expanded(child: _sendBtn('Send LFAC',
                  c ? () => state.send('LFAC=${lfac.toStringAsFixed(3)}') : null)),
              const SizedBox(width: 8),
              Expanded(child: _sendBtn('Send RFAC',
                  c ? () => state.send('RFAC=${rfac.toStringAsFixed(3)}') : null)),
            ]),
          ]),

          // ── Forward Navigation ────────────────────────────────────
          _section('Forward Navigation', [
            _slider('Dead-band MB (cm)', mb, 0, 10, (v) => setState(() => mb = v)),
            _sendBtn('Send MB',
                c ? () => state.send('MB=${mb.toStringAsFixed(2)}') : null),
            _slider('Sensitivity (cm)', sensitivity, 1, 50,
                (v) => setState(() => sensitivity = v)),
            _sendBtn('Send Sensitivity',
                c ? () => state.send('SENS=${sensitivity.toStringAsFixed(3)}') : null),
            _slider('Front Decel (cm)', frontDecel, 10, 100,
                (v) => setState(() => frontDecel = v)),
            _sendBtn('Send Front Decel',
                c ? () => state.send('FDEC=${frontDecel.toStringAsFixed(2)}') : null),
            _slider('Front Stop (cm)', frontStop, 5, 60,
                (v) => setState(() => frontStop = v)),
            _sendBtn('Send Front Stop',
                c ? () => state.send('FSTOP=${frontStop.toStringAsFixed(2)}') : null),
          ]),

          // ── Wall Detection ────────────────────────────────────────
          _section('Wall Detection', [
            _slider('D Far (cm)', dFar, 10, 80,
                (v) => setState(() => dFar = v)),
            _sendBtn('Send D Far',
                c ? () => state.send('DFAR=${dFar.toStringAsFixed(2)}') : null),
            _slider('Open Wall (cm)', openWall, 5, 60,
                (v) => setState(() => openWall = v)),
            _sendBtn('Send Open Wall',
                c ? () => state.send('OPWALL=${openWall.toStringAsFixed(2)}') : null),
            _slider('Desired Wall (cm)', desiredWall, 5, 40,
                (v) => setState(() => desiredWall = v)),
            _sendBtn('Send Desired Wall',
                c ? () => state.send('DWALL=${desiredWall.toStringAsFixed(2)}') : null),
            _slider('EMA Alpha', ema, 0.01, 1.0,
                (v) => setState(() => ema = v)),
            _sendBtn('Send EMA',
                c ? () => state.send('EMA=${ema.toStringAsFixed(3)}') : null),
          ]),

          // ── Turn PD ───────────────────────────────────────────────
          _section('Turn PD', [
            _slider('Turn Kp', turnKp, 0, 20, (v) => setState(() => turnKp = v)),
            _slider('Turn Kd', turnKd, 0, 5, (v) => setState(() => turnKd = v)),
            Row(children: [
              Expanded(child: _sendBtn('Send TKP',
                  c ? () => state.send('TKP=${turnKp.toStringAsFixed(3)}') : null)),
              const SizedBox(width: 8),
              Expanded(child: _sendBtn('Send TKD',
                  c ? () => state.send('TKD=${turnKd.toStringAsFixed(3)}') : null)),
            ]),
            _slider('Turn Angle (°)', turnAngleDeg, 45, 180,
                (v) => setState(() => turnAngleDeg = v)),
            _sendBtn('Send Turn Angle',
                c ? () => state.send('TANG=${turnAngleDeg.toStringAsFixed(2)}') : null),
            _slider('Stop Threshold (°)', turnStopDeg, 0.5, 10,
                (v) => setState(() => turnStopDeg = v)),
            _sendBtn('Send Stop Threshold',
                c ? () => state.send('TSTOP=${turnStopDeg.toStringAsFixed(2)}') : null),
            _slider('Timeout (ms)', turnTimeoutMs, 1000, 15000,
                (v) => setState(() => turnTimeoutMs = v), divisions: 140),
            _sendBtn('Send Timeout',
                c ? () => state.send('TTOUT=${turnTimeoutMs.round()}') : null),
          ]),

          // ── Turn Speed Limits ─────────────────────────────────────
          _section('Turn Speed Limits', [
            _slider('Max Speed (%)', turnMaxSpeed, 10, 100,
                (v) => setState(() => turnMaxSpeed = v), divisions: 90),
            _sendBtn('Send Max Speed',
                c ? () => state.send('TMAXSPD=${turnMaxSpeed.round()}') : null),
            _slider('Min Speed (%)', turnMinSpeed, 5, 80,
                (v) => setState(() => turnMinSpeed = v), divisions: 75),
            _sendBtn('Send Min Speed',
                c ? () => state.send('TMINSPD=${turnMinSpeed.round()}') : null),
          ]),

          // ── Turn Approach Zone ────────────────────────────────────
          _section('Turn Approach Zone', [
            _slider('Approach Threshold (°)', turnApproachDeg, 2, 45,
                (v) => setState(() => turnApproachDeg = v)),
            _sendBtn('Send Approach Threshold',
                c ? () => state.send('TAPPDEG=${turnApproachDeg.toStringAsFixed(2)}') : null),
            _slider('Approach Max PWM (%)', turnApproachMax, 5, 80,
                (v) => setState(() => turnApproachMax = v), divisions: 75),
            _sendBtn('Send Approach Max',
                c ? () => state.send('TAPPMAX=${turnApproachMax.round()}') : null),
            _slider('Approach Min PWM (%)', turnApproachMin, 5, 60,
                (v) => setState(() => turnApproachMin = v), divisions: 55),
            _sendBtn('Send Approach Min',
                c ? () => state.send('TAPPMIN=${turnApproachMin.round()}') : null),
          ]),

          const SizedBox(height: 8),
          OutlinedButton.icon(
            icon: const Icon(Icons.refresh),
            label: const Text('Pull current values (GET)'),
            onPressed: c
                ? () async {
                    await state.send('GET');
                    setState(() => _seeded = false);
                  }
                : null,
          ),
          const SizedBox(height: 16),
        ],
      ),
    );
  }

  Widget _section(String title, List<Widget> children) {
    return Card(
      margin: const EdgeInsets.only(bottom: 8),
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
          width: 120,
          child: Text(label, style: const TextStyle(fontSize: 12)),
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
          width: 52,
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
      padding: const EdgeInsets.only(top: 4, bottom: 2),
      child: FilledButton(onPressed: onPressed, child: Text(label)),
    );
  }
}
