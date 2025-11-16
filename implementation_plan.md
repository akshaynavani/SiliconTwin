# SiliconTwin — Implementation Plan

**Firmware Simulator & Hardware-in-the-Loop Test Framework**

Tech: C (C11), C++17, Python 3.11+, CMake, Google Test.

## Goal

Run real power-electronics control firmware (a digitally controlled buck converter) on a
simulated microcontroller so it can be validated without physical hardware. The firmware is
written strictly against a small HAL (`hal.h`); the simulator provides an implementation of that
HAL backed by emulated peripherals (UART, GPIO, CAN, ADC, PWM, watchdog) and an averaged
analog plant model. A Python harness drives the simulator across many firmware configurations
and fault scenarios as an automated regression suite.

## Key design decisions

- **Deterministic, discrete-time simulation.** A single virtual clock in microseconds. The sim
  steps in 10 us ticks; the plant is integrated with 1 us Euler sub-steps; the firmware control
  ISR fires at 10 kHz and the main-loop task at 1 kHz. Noise uses a seeded xorshift PRNG so every
  run is reproducible.
- **Firmware/simulator split via a HAL.** `firmware/` never includes simulator headers. The
  `hal_sim` layer binds HAL calls to an `mcu_t` instance, which is exactly the seam a real board
  port would replace.
- **Peripheral fidelity where it matters for testing:** UART byte timing from baud rate and
  FIFO overrun; GPIO direction/pull/edge interrupts; CAN 11-bit frames with ID arbitration,
  acceptance filters, RX FIFO, TEC/REC error counters and error-passive/bus-off states; an
  independent watchdog that resets the MCU and records the reset cause; retained (no-init) RAM
  that survives warm resets.
- **Fault injection** is first-class: overcurrent/short, input over/under-voltage, sensor stuck,
  CAN bus error rate and bus-off, UART line noise, firmware hang (to trip the watchdog).
- **Text interfaces:** INI config files + time-stamped scenario scripts in, JSON summary and
  optional CSV trace out. That keeps the Python harness simple and language-agnostic.

## Phases

### Phase 1 — Simulation core and basic peripherals
Deliverables: CMake build, virtual clock, deterministic PRNG, GPIO and UART emulation, the HAL
interface the firmware targets.
Files: `CMakeLists.txt`, `sim/include/sim/{clock,rng,gpio,uart}.h`, `sim/src/{clock,rng,gpio,uart}.c`,
`firmware/include/fw/hal.h`.

### Phase 2 — CAN bus, ADC/PWM, watchdog, MCU aggregate, plant model
Deliverables: CAN bus emulation (arbitration, filters, error states), ADC + PWM peripherals,
independent watchdog with reset-cause and retained RAM, `mcu_t` aggregate with reset logic,
averaged buck-converter plant, fault-injection model.
Files: `sim/include/sim/{can,adc,pwm,watchdog,mcu,plant,faults}.h`, matching `sim/src/*.c`.

### Phase 3 — Firmware and simulator runner
Deliverables: buck-converter firmware (PI voltage loop with current limit, soft start,
protection state machine with auto-retry/latch, CAN telemetry + command handling, UART command
shell, watchdog servicing), `hal_sim` binding, INI config + scenario parsers, `silicontwin` CLI
emitting JSON metrics and CSV traces, sample configs/scenarios.
Files: `firmware/include/fw/*.h`, `firmware/src/*.c`, `sim/src/hal_sim.c`,
`sim/include/sim/{config,scenario,runner,metrics}.h` + sources, `app/main.c`,
`configs/*.ini`, `scenarios/*.scn`.

### Phase 4 — Google Test suites
Deliverables: unit tests for every peripheral, CAN arbitration/error state tests, watchdog
reset/recovery tests, firmware-in-the-loop tests (startup regulation, load step, OCP trip and
retry, latch-off, watchdog recovery from a hang, CAN setpoint commands, UART shell).
Files: `tests/CMakeLists.txt`, `tests/test_*.cpp`, `tests/sim_fixture.h`.

### Phase 5 — Python regression harness
Deliverables: `silicontwin_harness` package (runner wrapping the CLI, TOML suite loader with
configuration matrices, expectation checker, parallel execution, JUnit XML + Markdown reports,
baseline comparison), regression suite definition, pytest tests for the harness.
Files: `harness/pyproject.toml`, `harness/silicontwin_harness/*.py`, `harness/tests/*.py`,
`regression/suite.toml`.

After the phases: `.gitignore` and `README.md` (architecture diagram, run and test instructions).
