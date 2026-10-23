# Ignition System Architecture

The ignition (spark) system determines when to charge (dwell) and fire (discharge) ignition coils based on engine position, RPM, and load.

## The Spark Pipeline

1.  **Timing Advance Calculation**: Core engine math in `algo/` determines the target advance (e.g., 20° BTDC) based on RPM and Load (MAP or TPS).
2.  **Dwell Calculation**: Determination of coil charging time (in milliseconds) based on battery voltage and RPM.
3.  **Angle Translation**: `engine_cycle/` translates the timing advance into a specific crank angle.
4.  **Event Generation**: Ignition events are created for each cylinder. An event consists of two actions: `START` (start dwelling) and `END` (fire spark).
5.  **Scheduling**: The `EventQueue` handles the execution of these actions at the precise crank angle or time.
6.  **Limp Management**: `LimpManager` can inhibit spark for RPM limiting, protection, or sync requirements.
7.  **Output Driver**: Coils are fired via digital outputs or specialized ignition drivers.

## Ignition Modes
- **Individual Coils (Sequential)**: One output per cylinder, fired every 720°.
- **Wasted Spark**: Outputs shared between companion cylinders, fired every 360°.
- **One Coil (Distributor)**: A single output fired for every cylinder.

## Key Files
- `firmware/controllers/engine_cycle/spark_logic.cpp`: Core scheduling and event generation.
- `firmware/controllers/algo/ignition/ignition_state.cpp`: Calculation of base timing and corrections (IAT, CLT, Knock).
- `firmware/controllers/engine_cycle/event_queue.cpp`: The high-precision scheduler.

## Rev-limit timing and ignition cuts

The [main rev limiter](rev_limiters.md) subtracts a relative timing retard
across the RPM hysteresis window when Cut spark on RPM limit is enabled and
timing mode is dynamic. It reaches maximum retard at the selected hard limit;
the separate full spark cut starts strictly above that limit. Fixed timing
bypasses this retard but does not bypass the hard cut. The guide covers
coolant-based limits, recovery boundaries and cut diagnostics.

## Launch timing and ignition cuts

[Launch control](launch_control.md) can replace running advance with an absolute
angle, optionally interpolated across the pre-launch RPM window. This value is
still subject to downstream timing corrections; fixed timing bypasses the
running-advance override. Launch spark skipping uses the hard limiter, reaching
100% at Launch RPM, while the separate `LimpManager` launch cut requires RPM
strictly above that target. See the launch guide for configuration, shared-input
flat-shift behavior and diagnostic boundaries.

## Burst knock retard

`BurstKnock` in `controllers/modules/burst_knock` uses the shared TPS AE detector
to request feed-forward retard without an acoustic knock event. The module runs
after `TpsAccelEnrichment::onNewValue()` in `Engine::periodicSlowCallback()`.
Its rate proxy is AE's largest adjacent TPS jump divided by
`max(tpsAccelLookback, 0.001)`, rather than an instantaneous derivative.
The selected AE fuel mode does not gate this timing correction.

While `isAboveAccelThreshold` is true, the module interpolates the 8×8 RPM/rate
table, clamps negative retard to zero, and resets the decay timer. Once false,
the last table value decays linearly from the last above-threshold sample.
Disabling clears the state on the next slow callback. Defaults are disabled,
zero table, one-second decay. Settings use the main configuration page.

`getAdvanceCorrections()` subtracts the result, preserving the existing fixed
timing bypass and cranking-correction switch. Telemetry is `isBurstKnockActive`,
`burstKnockRetard`, and `burstKnockTpsRate`. Tests in
`unit_tests/tests/ignition_injection/test_burst_knock.cpp` exercise the detector,
table, decay and ignition integration.

## Tooth-locked cranking spark

`crankingToothLockedSpark` (default off) fires each spark on a trigger tooth
while the engine is CRANKING, instead of timing it from the stored RPM, which
is only refreshed once per cycle. It is meant for sparse wheels such as a
12 tooth crank, where cranking speed swings widely within a cycle.

The spark tooth is the first tooth at or after the commanded spark, so the
spark is never earlier than commanded. It is used when that tooth is BTDC and
at most `crankingToothLockMaxRetardDeg` (default 5) after the command; other
events keep normal scheduling (`crankSparkNotLockable`). The path is decided
once per spark, on the first tooth the mode allows, and kept until the spark
fires. The charge start is predicted from `ToothPeriodWindow` tooth intervals.
If the coil has not started charging when the spark tooth arrives, it charges
then and fires up to `crankingToothLateSparkMaxDeg` degrees late (default 15;
0 skips the spark, use it with a mechanical rotor distributor), shown by
`Ign: Spark late (tooth-lock)`.
If the releasing tooth never arrives, the coil is released after the larger of
3 x dwell and `crankingToothLockBackupDwellMs` (default 40 ms) while cranking
(the smaller once RUNNING), which can be earlier than commanded. A setting of 0
means the default, as in a tune saved before the settings existed.

The wheel needs rising edges only and a tooth exactly one stroke after every
tooth, with 3 to 60 teeth per stroke, on a four stroke engine with even
firing (`CrankToothSpark::isGeometryValid`); 36-1 does not qualify.
`CrankToothSpark` is an `EngineModule` (`crank_tooth_spark.cpp`); the tooth
history is in `tooth_period_window.h`, the scheduling hook in `spark_logic.cpp`,
tests in `test_crank_tooth*.cpp`.

`camEarlyRiseLatch` (default off): with a single tooth cam on a crank speed
trigger, a cam rise between crank sync and the first RPM is kept as its sync
counter and gives the phase right after the first RPM (`camEarlyRiseUsed`),
so phase-required ignition starts a cam revolution sooner (`TriggerCentral`).

## Implementation Notes
- **BTDC vs ATDC**: Advance is typically measured in degrees Before Top Dead Center (BTDC). Internal math may use 0-720° cycle coordinates.
- **Dwell Overlap**: rusEFI supports overlapping dwell events (where one coil starts charging before another finishes firing).
- **Wasted Spark on Odd Fire**: Specialized logic in `spark_logic.cpp` handles uneven firing intervals.

Use coilState1 coilState2 etc in logs (see unit_tests/test_results/readme.md for unit tests output) to
assert actual logic state.
