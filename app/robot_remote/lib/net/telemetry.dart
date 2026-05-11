/// Parsed snapshot of any ack the firmware emits.
///
/// Field order follows the `GET` response in src/simple_fsm_main.c:274-283:
///   KP KI KD MB SPD LFAC RFAC HKP HKD WG ANG TGT RUN L F R
/// All fields are nullable so a partial ack (e.g. `OK SPEED=42`) only
/// updates what it actually carries.
class Telemetry {
  final double? kp, ki, kd, mb;
  final int? spd;
  final double? lfac, rfac;
  final double? hkp, hkd, wg;
  final double? angle, target;
  final int? running;
  final double? left, front, right;
  final int? state;
  final int? tcnt;
  final String? tseq;

  const Telemetry({
    this.kp,
    this.ki,
    this.kd,
    this.mb,
    this.spd,
    this.lfac,
    this.rfac,
    this.hkp,
    this.hkd,
    this.wg,
    this.angle,
    this.target,
    this.running,
    this.left,
    this.front,
    this.right,
    this.state,
    this.tcnt,
    this.tseq,
  });

  /// Merge a fresh ack on top of the previous snapshot — null fields keep
  /// the prior value, so the dashboard doesn't blank out between polls.
  Telemetry merge(Telemetry next) => Telemetry(
        kp: next.kp ?? kp,
        ki: next.ki ?? ki,
        kd: next.kd ?? kd,
        mb: next.mb ?? mb,
        spd: next.spd ?? spd,
        lfac: next.lfac ?? lfac,
        rfac: next.rfac ?? rfac,
        hkp: next.hkp ?? hkp,
        hkd: next.hkd ?? hkd,
        wg: next.wg ?? wg,
        angle: next.angle ?? angle,
        target: next.target ?? target,
        running: next.running ?? running,
        left: next.left ?? left,
        front: next.front ?? front,
        right: next.right ?? right,
        state: next.state ?? state,
        tcnt: next.tcnt ?? tcnt,
        tseq: next.tseq ?? tseq,
      );

  static double? _f(RegExpMatch? m) =>
      m == null ? null : double.tryParse(m.group(1)!);
  static int? _i(RegExpMatch? m) =>
      m == null ? null : int.tryParse(m.group(1)!);

  /// Picks every `KEY=value` token out of `reply`. Tolerates extra prose
  /// (e.g. `OK AUTO ang=12.3`) because we anchor on `\b<key>=`.
  static Telemetry parse(String reply) {
    RegExpMatch? f(String key) =>
        RegExp('\\b$key=(-?\\d+(?:\\.\\d+)?)', caseSensitive: false)
            .firstMatch(reply);
    return Telemetry(
      kp: _f(f('KP')),
      ki: _f(f('KI')),
      kd: _f(f('KD')),
      mb: _f(f('MB')),
      spd: _i(f('SPD')) ?? _i(f('SPEED')),
      lfac: _f(f('LFAC')),
      rfac: _f(f('RFAC')),
      hkp: _f(f('HKP')),
      hkd: _f(f('HKD')),
      wg: _f(f('WG')),
      angle: _f(f('ANG')) ?? _f(f('ang')),
      target: _f(f('TGT')),
      running: _i(f('RUN')),
      left: _f(f('L')),
      front: _f(f('F')),
      right: _f(f('R')),
      state: _i(f('STATE')),
      tcnt: _i(f('TCNT')),
      tseq: RegExp(r'\bTSEQ=([A-Z]+)', caseSensitive: false)
          .firstMatch(reply)
          ?.group(1),
    );
  }
}
