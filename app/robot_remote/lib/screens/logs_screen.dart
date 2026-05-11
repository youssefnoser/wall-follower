import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../state/robot_state.dart';

class LogsScreen extends StatefulWidget {
  const LogsScreen({super.key});

  @override
  State<LogsScreen> createState() => _LogsScreenState();
}

class _LogsScreenState extends State<LogsScreen> {
  final _scroll = ScrollController();
  bool _autoScroll = true;
  int _lastLen = 0;

  @override
  void initState() {
    super.initState();
    _scroll.addListener(() {
      if (!_scroll.hasClients) return;
      final atBottom =
          _scroll.position.pixels >= _scroll.position.maxScrollExtent - 4;
      if (_autoScroll != atBottom) {
        setState(() => _autoScroll = atBottom);
      }
    });
  }

  @override
  void dispose() {
    _scroll.dispose();
    super.dispose();
  }

  String _ts(DateTime t) =>
      '${t.hour.toString().padLeft(2, '0')}:${t.minute.toString().padLeft(2, '0')}:${t.second.toString().padLeft(2, '0')}.${(t.millisecond ~/ 100)}';

  @override
  Widget build(BuildContext context) {
    final state = context.watch<RobotState>();
    final entries = state.log.toList(growable: false);

    if (_autoScroll && entries.length != _lastLen) {
      _lastLen = entries.length;
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (_scroll.hasClients) {
          _scroll.jumpTo(_scroll.position.maxScrollExtent);
        }
      });
    }

    return SafeArea(
      child: Column(
        children: [
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
            child: Row(
              children: [
                Text('${entries.length} entries',
                    style: const TextStyle(color: Colors.grey)),
                const Spacer(),
                TextButton.icon(
                  icon: Icon(_autoScroll
                      ? Icons.lock_outline
                      : Icons.lock_open_outlined),
                  label: Text(_autoScroll ? 'Follow' : 'Free scroll'),
                  onPressed: () {
                    setState(() => _autoScroll = !_autoScroll);
                    if (_autoScroll && _scroll.hasClients) {
                      _scroll.jumpTo(_scroll.position.maxScrollExtent);
                    }
                  },
                ),
                IconButton(
                  icon: const Icon(Icons.delete_outline),
                  tooltip: 'Clear',
                  onPressed: state.clearLog,
                ),
              ],
            ),
          ),
          const Divider(height: 1),
          Expanded(
            child: entries.isEmpty
                ? const Center(
                    child: Text('No log entries yet.',
                        style: TextStyle(color: Colors.grey)))
                : ListView.builder(
                    controller: _scroll,
                    itemCount: entries.length,
                    itemBuilder: (_, i) {
                      final e = entries[i];
                      final col =
                          e.tx ? Colors.blue.shade700 : Colors.grey.shade800;
                      return Padding(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 12, vertical: 2),
                        child: RichText(
                          text: TextSpan(
                            style: const TextStyle(
                                fontFamily: 'monospace',
                                fontSize: 12,
                                color: Colors.black),
                            children: [
                              TextSpan(
                                  text: '${_ts(e.ts)} ',
                                  style: const TextStyle(color: Colors.grey)),
                              TextSpan(
                                  text: e.tx ? '→ ' : '← ',
                                  style: TextStyle(
                                      color: col,
                                      fontWeight: FontWeight.bold)),
                              TextSpan(
                                  text: e.text, style: TextStyle(color: col)),
                            ],
                          ),
                        ),
                      );
                    },
                  ),
          ),
        ],
      ),
    );
  }
}
