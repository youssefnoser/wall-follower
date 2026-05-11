import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../state/robot_state.dart';

class ManualScreen extends StatelessWidget {
  const ManualScreen({super.key});

  @override
  Widget build(BuildContext context) {
    final state = context.watch<RobotState>();
    final enabled = state.connected && state.mode == DriveMode.manual;

    return SafeArea(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            if (!state.connected)
              const _Banner(
                text: 'Not connected. Connect on the Dashboard tab.',
                color: Colors.red,
              ),
            if (state.connected && state.mode == DriveMode.auto)
              const _Banner(
                text: 'AUTO mode active. Switch to MANUAL on the Dashboard.',
                color: Colors.orange,
              ),
            const Spacer(),
            _row([_DirBtn('FWD', Icons.arrow_upward, enabled,
                () => state.send('FWD'))]),
            const SizedBox(height: 12),
            _row([
              _DirBtn('LEFT', Icons.arrow_back, enabled,
                  () => state.send('LEFT')),
              _DirBtn('STOP', Icons.stop, state.connected,
                  () => state.send('STOP'),
                  color: Colors.red),
              _DirBtn('RIGHT', Icons.arrow_forward, enabled,
                  () => state.send('RIGHT')),
            ]),
            const SizedBox(height: 12),
            _row([_DirBtn('BWD', Icons.arrow_downward, enabled,
                () => state.send('BWD'))]),
            const Spacer(),
            Text('Mode: ${state.mode.name.toUpperCase()}',
                style: const TextStyle(color: Colors.grey)),
          ],
        ),
      ),
    );
  }

  Widget _row(List<Widget> children) {
    final spaced = <Widget>[];
    for (var i = 0; i < children.length; i++) {
      if (i > 0) spaced.add(const SizedBox(width: 12));
      spaced.add(children[i]);
    }
    return Row(mainAxisAlignment: MainAxisAlignment.center, children: spaced);
  }
}

class _DirBtn extends StatelessWidget {
  final String label;
  final IconData icon;
  final bool enabled;
  final VoidCallback onTap;
  final Color? color;
  const _DirBtn(this.label, this.icon, this.enabled, this.onTap, {this.color});

  @override
  Widget build(BuildContext context) {
    final c = color ?? Theme.of(context).colorScheme.primary;
    return SizedBox(
      width: 96,
      height: 96,
      child: FilledButton(
        style: FilledButton.styleFrom(
          backgroundColor: enabled ? c : Colors.grey.shade400,
          padding: EdgeInsets.zero,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        ),
        onPressed: enabled ? onTap : null,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(icon, size: 32),
            const SizedBox(height: 4),
            Text(label, style: const TextStyle(fontWeight: FontWeight.bold)),
          ],
        ),
      ),
    );
  }
}

class _Banner extends StatelessWidget {
  final String text;
  final Color color;
  const _Banner({required this.text, required this.color});

  @override
  Widget build(BuildContext context) {
    return Container(
      width: double.infinity,
      margin: const EdgeInsets.only(bottom: 8),
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.12),
        border: Border.all(color: color),
        borderRadius: BorderRadius.circular(8),
      ),
      child: Text(text, style: TextStyle(color: color)),
    );
  }
}
