import 'dart:async';
import 'dart:collection';

import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

import '../net/robot_client.dart';
import '../net/telemetry.dart';

enum DriveMode { auto, manual }

class LogEntry {
  final DateTime ts;
  final bool tx; // true = sent, false = received
  final String text;
  LogEntry(this.tx, this.text) : ts = DateTime.now();
}

class FsmEvent {
  final DateTime ts;
  final String from;
  final String to;
  final String reason;
  FsmEvent(this.from, this.to, this.reason) : ts = DateTime.now();
}

class RobotState extends ChangeNotifier {
  static const _kHost = 'host';
  static const _kPort = 'port';
  static const int defaultPort = 1234;
  static const int _maxLog = 500;
  static const _pollPeriod = Duration(milliseconds: 500);

  final RobotClient _client = RobotClient();
  final Queue<LogEntry> _log = Queue<LogEntry>();
  final Queue<FsmEvent> _fsmEvents = Queue<FsmEvent>();
  StreamSubscription<String>? _linesSub;
  Timer? _poll;
  bool _pollInflight = false;

  String host = '';
  int port = defaultPort;
  Telemetry telemetry = const Telemetry();
  DriveMode mode = DriveMode.auto;

  RobotState() {
    _client.connected.addListener(notifyListeners);
    _linesSub = _client.lines.listen(_onLine);
    _loadPrefs();
  }

  bool get connected => _client.connected.value;
  Iterable<LogEntry> get log => _log;
  Iterable<FsmEvent> get fsmEvents => _fsmEvents;

  Future<void> _loadPrefs() async {
    final p = await SharedPreferences.getInstance();
    host = p.getString(_kHost) ?? '';
    port = p.getInt(_kPort) ?? defaultPort;
    notifyListeners();
  }

  Future<void> _savePrefs() async {
    final p = await SharedPreferences.getInstance();
    await p.setString(_kHost, host);
    await p.setInt(_kPort, port);
  }

  Future<bool> connect(String h, int p) async {
    host = h;
    port = p;
    _append(LogEntry(true, '[connect $h:$p]'));
    final ok = await _client.connect(h, p);
    _append(LogEntry(false, ok ? '[connected]' : '[connect failed]'));
    if (ok) {
      await _savePrefs();
      _startPolling();
      // Pull full state once so the PID screen can seed from real values.
      await send('STATUS');
    }
    return ok;
  }

  Future<void> disconnect() async {
    _stopPolling();
    await _client.disconnect();
    _append(LogEntry(false, '[disconnected]'));
  }

  Future<void> send(String cmd) async {
    if (!connected) {
      _append(LogEntry(true, '$cmd  (not connected)'));
      return;
    }
    _append(LogEntry(true, cmd));
    await _client.send(cmd);
  }

  /// Convenience: switch the dashboard mode and tell the firmware about it.
  /// AUTO resumes wall-following; MANUAL just stops the robot — the user then
  /// picks a direction from the Manual screen.
  Future<void> setMode(DriveMode m) async {
    mode = m;
    notifyListeners();
    await send(m == DriveMode.auto ? 'AUTO' : 'STOP');
  }

  void clearLog() {
    _log.clear();
    _fsmEvents.clear();
    notifyListeners();
  }

  List<String> get turns {
    final seq = telemetry.tseq;
    if (seq == null || seq.isEmpty || seq == '-') return const [];
    return seq.split('');
  }

  Future<void> clearTurns() => send('CLRTURNS');

  void _onLine(String line) {
    _append(LogEntry(false, line));

    if (line.startsWith('EVENT FSM=')) {
      final reg = RegExp(r'EVENT FSM=(\w+)->(\w+) REASON=(.+)');
      final match = reg.firstMatch(line);
      if (match != null) {
        final ev = FsmEvent(match.group(1)!, match.group(2)!, match.group(3)!);
        _fsmEvents.addLast(ev);
        if (_fsmEvents.length > 100) _fsmEvents.removeFirst();
      }
    }

    telemetry = telemetry.merge(Telemetry.parse(line));
    notifyListeners();
  }

  void _append(LogEntry e) {
    _log.addLast(e);
    while (_log.length > _maxLog) {
      _log.removeFirst();
    }
    notifyListeners();
  }

  void _startPolling() {
    _stopPolling();
    _poll = Timer.periodic(_pollPeriod, (_) async {
      if (!connected || _pollInflight) return;
      _pollInflight = true;
      try {
        await _client.send('GET');
      } finally {
        _pollInflight = false;
      }
    });
  }

  void _stopPolling() {
    _poll?.cancel();
    _poll = null;
  }

  @override
  void dispose() {
    _stopPolling();
    _linesSub?.cancel();
    _client.connected.removeListener(notifyListeners);
    _client.dispose();
    super.dispose();
  }
}
