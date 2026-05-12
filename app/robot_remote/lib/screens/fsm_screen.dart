import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../state/robot_state.dart';

class FsmScreen extends StatelessWidget {
  const FsmScreen({super.key});

  String _ts(DateTime t) =>
      '${t.hour.toString().padLeft(2, '0')}:${t.minute.toString().padLeft(2, '0')}:${t.second.toString().padLeft(2, '0')}';

  @override
  Widget build(BuildContext context) {
    final state = context.watch<RobotState>();
    final events = state.fsmEvents.toList().reversed.toList();

    return SafeArea(
      child: Column(
        children: [
          Padding(
            padding: const EdgeInsets.all(12),
            child: Row(
              children: [
                const Icon(Icons.history, color: Colors.grey),
                const SizedBox(width: 8),
                Text('FSM Transition Log',
                    style: Theme.of(context).textTheme.titleMedium),
                const Spacer(),
                Text('${events.length} events',
                    style: const TextStyle(color: Colors.grey, fontSize: 12)),
              ],
            ),
          ),
          const Divider(height: 1),
          Expanded(
            child: events.isEmpty
                ? const Center(
                    child: Text('No state transitions recorded.',
                        style: TextStyle(color: Colors.grey)))
                : ListView.separated(
                    padding: const EdgeInsets.all(12),
                    itemCount: events.length,
                    separatorBuilder: (_, __) => const SizedBox(height: 8),
                    itemBuilder: (context, i) {
                      final ev = events[i];
                      return _EventCard(ev: ev, ts: _ts(ev.ts));
                    },
                  ),
          ),
        ],
      ),
    );
  }
}

class _EventCard extends StatelessWidget {
  final FsmEvent ev;
  final String ts;

  const _EventCard({required this.ev, required this.ts});

  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 0,
      color: Theme.of(context).colorScheme.surfaceContainerHighest.withValues(alpha: 0.3),
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(12),
        side: BorderSide(
          color: Theme.of(context).colorScheme.outlineVariant,
          width: 0.5,
        ),
      ),
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Text(ts,
                    style: const TextStyle(
                        fontFamily: 'monospace',
                        fontSize: 12,
                        color: Colors.grey)),
                const Spacer(),
                Container(
                  padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
                  decoration: BoxDecoration(
                    color: Colors.blue.withValues(alpha: 0.1),
                    borderRadius: BorderRadius.circular(4),
                  ),
                  child: Text(
                    ev.reason,
                    style: const TextStyle(
                        fontSize: 10,
                        fontWeight: FontWeight.bold,
                        color: Colors.blue),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 8),
            Row(
              children: [
                _StateLabel(label: ev.from, isDim: true),
                const Padding(
                  padding: EdgeInsets.symmetric(horizontal: 8),
                  child: Icon(Icons.arrow_forward, size: 16, color: Colors.grey),
                ),
                _StateLabel(label: ev.to, isDim: false),
              ],
            ),
          ],
        ),
      ),
    );
  }
}

class _StateLabel extends StatelessWidget {
  final String label;
  final bool isDim;

  const _StateLabel({required this.label, required this.isDim});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
      decoration: BoxDecoration(
        color: isDim ? Colors.grey.withValues(alpha: 0.1) : Colors.indigo.withValues(alpha: 0.1),
        borderRadius: BorderRadius.circular(6),
        border: Border.all(
          color: isDim ? Colors.grey.withValues(alpha: 0.2) : Colors.indigo.withValues(alpha: 0.3),
        ),
      ),
      child: Text(
        label,
        style: TextStyle(
          fontSize: 13,
          fontWeight: isDim ? FontWeight.normal : FontWeight.bold,
          color: isDim ? Colors.grey.shade700 : Colors.indigo,
        ),
      ),
    );
  }
}
