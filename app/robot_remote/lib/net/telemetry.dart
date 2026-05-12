/// Parsed snapshot of any ack the firmware emits.
///
/// All fields are nullable so a partial ack (e.g. `OK SPEED=42`) only
/// updates what it actually carries.
class Telemetry {
  // Heading-hold PD
  final double? hkp, hkd, wg, wgd;
  // Dead-band margin & base speed
  final double? mb;
  final int? spd;
  // Motor scale factors
  final double? lfac, rfac;
  // IMU state
  final double? angle, target;
  // FSM status
  final int? running, state;
  // Turn history
  final int? tcnt;
  final String? tseq;
  // Sensor readings
  final double? left, front, right;
  // EMA & wall-follow
  final double? ema, sensitivity, desiredWall, openWall, dFar;
  final int? speedFloor;
  // Front obstacle thresholds
  final double? frontDecel, frontStop;
  // Turn PD
  final double? turnKp, turnKd, turnStopDeg, turnAngleDeg, turnApproachDeg;
  final int? turnMaxSpeed, turnMinSpeed, turnTimeoutMs;
  final int? turnApproachMax, turnApproachMin;

  const Telemetry({
    this.hkp,
    this.hkd,
    this.wg,
    this.wgd,
    this.mb,
    this.spd,
    this.lfac,
    this.rfac,
    this.angle,
    this.target,
    this.running,
    this.state,
    this.tcnt,
    this.tseq,
    this.left,
    this.front,
    this.right,
    this.ema,
    this.sensitivity,
    this.desiredWall,
    this.openWall,
    this.dFar,
    this.speedFloor,
    this.frontDecel,
    this.frontStop,
    this.turnKp,
    this.turnKd,
    this.turnStopDeg,
    this.turnAngleDeg,
    this.turnApproachDeg,
    this.turnMaxSpeed,
    this.turnMinSpeed,
    this.turnTimeoutMs,
    this.turnApproachMax,
    this.turnApproachMin,
  });

  Telemetry merge(Telemetry next) => Telemetry(
        hkp: next.hkp ?? hkp,
        hkd: next.hkd ?? hkd,
        wg: next.wg ?? wg,
        wgd: next.wgd ?? wgd,
        mb: next.mb ?? mb,
        spd: next.spd ?? spd,
        lfac: next.lfac ?? lfac,
        rfac: next.rfac ?? rfac,
        angle: next.angle ?? angle,
        target: next.target ?? target,
        running: next.running ?? running,
        state: next.state ?? state,
        tcnt: next.tcnt ?? tcnt,
        tseq: next.tseq ?? tseq,
        left: next.left ?? left,
        front: next.front ?? front,
        right: next.right ?? right,
        ema: next.ema ?? ema,
        sensitivity: next.sensitivity ?? sensitivity,
        desiredWall: next.desiredWall ?? desiredWall,
        openWall: next.openWall ?? openWall,
        dFar: next.dFar ?? dFar,
        speedFloor: next.speedFloor ?? speedFloor,
        frontDecel: next.frontDecel ?? frontDecel,
        frontStop: next.frontStop ?? frontStop,
        turnKp: next.turnKp ?? turnKp,
        turnKd: next.turnKd ?? turnKd,
        turnStopDeg: next.turnStopDeg ?? turnStopDeg,
        turnAngleDeg: next.turnAngleDeg ?? turnAngleDeg,
        turnApproachDeg: next.turnApproachDeg ?? turnApproachDeg,
        turnMaxSpeed: next.turnMaxSpeed ?? turnMaxSpeed,
        turnMinSpeed: next.turnMinSpeed ?? turnMinSpeed,
        turnTimeoutMs: next.turnTimeoutMs ?? turnTimeoutMs,
        turnApproachMax: next.turnApproachMax ?? turnApproachMax,
        turnApproachMin: next.turnApproachMin ?? turnApproachMin,
      );

  static double? _f(RegExpMatch? m) =>
      m == null ? null : double.tryParse(m.group(1)!);
  static int? _i(RegExpMatch? m) =>
      m == null ? null : int.tryParse(m.group(1)!);

  static Telemetry parse(String reply) {
    RegExpMatch? f(String key) =>
        RegExp('\\b$key=(-?\\d+(?:\\.\\d+)?)', caseSensitive: false)
            .firstMatch(reply);
    return Telemetry(
      hkp: _f(f('HKP')),
      hkd: _f(f('HKD')),
      wg: _f(f('WG')),
      wgd: _f(f('WGD')),
      mb: _f(f('MB')),
      spd: _i(f('SPD')) ?? _i(f('SPEED')),
      lfac: _f(f('LFAC')),
      rfac: _f(f('RFAC')),
      angle: _f(f('ANG')) ?? _f(f('ang')),
      target: _f(f('TGT')),
      running: _i(f('RUN')),
      state: _i(f('STATE')),
      tcnt: _i(f('TCNT')),
      tseq: RegExp(r'\bTSEQ=([A-Z-]+)', caseSensitive: false)
          .firstMatch(reply)
          ?.group(1),
      left: _f(f('L')),
      front: _f(f('F')),
      right: _f(f('R')),
      ema: _f(f('EMA')),
      sensitivity: _f(f('SENS')),
      desiredWall: _f(f('DWALL')),
      openWall: _f(f('OPWALL')),
      dFar: _f(f('DFAR')),
      speedFloor: _i(f('SFLOOR')),
      frontDecel: _f(f('FDEC')),
      frontStop: _f(f('FSTOP')),
      turnKp: _f(f('TKP')),
      turnKd: _f(f('TKD')),
      turnStopDeg: _f(f('TSTOP')),
      turnAngleDeg: _f(f('TANG')),
      turnApproachDeg: _f(f('TAPPDEG')),
      turnMaxSpeed: _i(f('TMAXSPD')),
      turnMinSpeed: _i(f('TMINSPD')),
      turnTimeoutMs: _i(f('TTOUT')),
      turnApproachMax: _i(f('TAPPMAX')),
      turnApproachMin: _i(f('TAPPMIN')),
    );
  }
}
