import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart';

/// TCP client for the STM32 robot's ESP8266 bridge (port 1234, line-based ASCII).
class RobotClient {
  Socket? _socket;
  final _lines = StreamController<String>.broadcast();
  final ValueNotifier<bool> connected = ValueNotifier<bool>(false);

  String _buf = '';
  Future<void> _sendChain = Future<void>.value();

  Stream<String> get lines => _lines.stream;

  Future<bool> connect(String host, int port,
      {Duration timeout = const Duration(seconds: 4)}) async {
    await disconnect();
    try {
      final s = await Socket.connect(host, port, timeout: timeout);
      s.setOption(SocketOption.tcpNoDelay, true);
      _socket = s;
      connected.value = true;
      s.listen(
        _onData,
        onError: (_) => disconnect(),
        onDone: disconnect,
        cancelOnError: true,
      );
      return true;
    } catch (_) {
      return false;
    }
  }

  Future<void> disconnect() async {
    final s = _socket;
    _socket = null;
    if (s != null) {
      try {
        await s.close();
      } catch (_) {}
      try {
        s.destroy();
      } catch (_) {}
    }
    _buf = '';
    if (connected.value) connected.value = false;
  }

  /// Sends `cmd` followed by `\n`. Sends are serialized so concurrent slider
  /// drags can't interleave bytes on the wire.
  Future<void> send(String cmd) {
    final next = _sendChain.then((_) async {
      final s = _socket;
      if (s == null) return;
      try {
        s.add(utf8.encode('${cmd.trim()}\n'));
        await s.flush();
      } catch (_) {
        await disconnect();
      }
    });
    _sendChain = next.catchError((_) {});
    return next;
  }

  void _onData(List<int> chunk) {
    _buf += utf8.decode(chunk, allowMalformed: true);
    while (true) {
      final i = _buf.indexOf('\n');
      if (i < 0) break;
      var line = _buf.substring(0, i);
      _buf = _buf.substring(i + 1);
      if (line.endsWith('\r')) line = line.substring(0, line.length - 1);
      if (line.isNotEmpty) _lines.add(line);
    }
  }

  void dispose() {
    disconnect();
    _lines.close();
    connected.dispose();
  }
}
