# Time Synchronization (GNSS + RTC)

## Overview

The eval-pqmon firmware can align its power quality measurement engine to an
absolute UTC time reference obtained from a GNSS receiver, and hold that
reference across GNSS outages using an on-board RTC.

The feature is compiled in with `TIME_SYNC=y` (which defines `PQM_TIME_SYNC`)
and is disabled by default, leaving the base project unchanged.

| Source               | Role                                   | Provides                                     |
| -------------------- | -------------------------------------- | -------------------------------------------- |
| GNSS (u-blox)        | Master reference when a fix is present | Absolute UTC time + 1PPS pulse               |
| RTC (MAX31343)       | Holdover when GNSS is lost             | Local 1PPS pulse, seeded from GNSS           |
| Capture timer (TMR1) | Common timebase                        | Timestamps both pulses for drift measurement |

## How it works

The subsystem is split into three cooperating layers under `src/common`:

- **`gnss_utils.c/.h`** — Brings up the GNSS receiver (baud negotiation via `gnss_init_set_baud`), parses the precise-time message, and exposes `gnss_precise_time` (UTC fields + Unix epoch).
- **`pps_utils.c/.h`** — PPS substrate: captures both 1PPS pulses against `TMR1`, validates pulse period, measures GNSS↔RTC drift, and runs the synchronization state machine (`time_sync_boot`,`pps_run_state_machine`).
- **`rtc_utils.c/.h`** — Prepares, resets and writes the MAX31343 so its 1PPS edge is realigned to the GNSS second boundary.

### Boot sequence

`time_sync_boot()` runs three phases, each bounded by its own timeout:

1. **Phase 1 — acquisition** (timeout `GNSS_BOOT_PHASE1_TIMEOUT_S`, 35 s).
   Waits for the first GNSS 1PPS edge carrying a valid fix and anchors `time_ms`
   to the GNSS epoch. If the timeout expires with no fix, the firmware **falls
   back to the RTC** (`rtc_get_time_burst`) for the absolute time and skips the
   GNSS-only phases. If the RTC read also fails, there is no time source and boot
   returns `-ETIMEDOUT`. The system therefore still comes up on the RTC alone
   when GNSS is unavailable (e.g. indoors).
2. **Phase 2 — frequency calibration** (timeout `GNSS_BOOT_PHASE2_TIMEOUT_S`,
   20 s; only if a GNSS PPS was seen). Counts `GNSS_CALIB_PPS_COUNT` (10) pulses
   and measures the elapsed `TMR1` ticks over the `(count − 1)` one-second
   intervals to derive the *real* capture-timer frequency (`pps_real_freq_hz`).
   This corrects for the MCU crystal's actual deviation from the nominal 60 MHz.
   If the measurement is incomplete or out of range, it falls back to the
   nominal frequency.
3. **Phase 3 — re-anchor** (timeout `GNSS_BOOT_PHASE3_TIMEOUT_S`, 5 s). Advances
   the time anchor by the seconds consumed during calibration so `time_ms` lands
   on the correct absolute second.

At run time the measured frequency keeps being refined by an EMA filter
(τ ≈ 16 s), so `pps_real_freq_hz` tracks slow crystal drift with temperature.

#### Tunable parameters

These are compile-time `#define`s in `src/common/common_data.h`:

| Define                       | Default | Meaning                                                |
| ---------------------------- | ------- | -------------------------------------------------------|
| `GNSS_UART_BAUDRATE`         | 115200  | GNSS module UART line rate                             |
| `GNSS_PPS_LENGTH`            | 500     | 1PPS high time (ms), used when configuring the RTC SQW |
| `SYNC_THRESHOLD_US`          | 10000   | Drift (µs) above which a resync is triggered           |
| `GNSS_BOOT_PHASE1_TIMEOUT_S` | 35      | Phase 1 fix-acquisition timeout (s)                    |
| `GNSS_BOOT_PHASE2_TIMEOUT_S` | 20      | Phase 2 calibration timeout (s)                        |
| `GNSS_BOOT_PHASE3_TIMEOUT_S` | 5       | Phase 3 re-anchor timeout (s)                          |
| `GNSS_CALIB_PPS_COUNT`       | 10      | PPS edges sampled for frequency calibration            |

> **Constraint:** keep `GNSS_BOOT_PHASE2_TIMEOUT_S` ≥ 2 × `GNSS_CALIB_PPS_COUNT`,
> otherwise calibration cannot collect enough edges before Phase 2 times out and
> the firmware silently falls back to the nominal frequency. A cold-boot GNSS fix
> can take up to ~60 s; `GNSS_BOOT_PHASE1_TIMEOUT_S` is the point past which the
> firmware stops waiting and boots on the RTC instead.

### Run time

`pps_run_state_machine()` runs continuously and compares the two captured
pulses:

- While GNSS is present, GNSS is the master reference.
- If GNSS is lost, the RTC holds the timebase (holdover) and the measured
  drift is exposed for monitoring.
- `pqm_update_timestamp()` stamps the metrology output with the current epoch.

The PPS state machine (`pps_state_t`) tracks which pulses have arrived in the
current second (`PPS_WAIT`, `PPS_WAIT_RTC`, `PPS_WAIT_GNSS`, `PPS_BOTH`,
`PPS_GNSS_MISS`, `PPS_RTC_MISS`, `PPS_ARMED`, `PPS_SYNC`), while the sync state
(`sync_state_t`: `SYNC_DONE`, `SYNC_PEND_WR`, `SYNC_SETTLE`) sequences the
RTC realignment.

> **Note:** The MAX31343 has no sub-second register, so its 1PPS phase can only
> be set indirectly. Writing the time registers re-locks the chip's internal
> divider one second later, shifting the RTC 1PPS edge by a fixed **~4 ms**
> transient; the reset that precedes each write costs a further **~2 ms**. To
> land the RTC edge on the GNSS second boundary, a dedicated one-shot timer
> (TMR2) fires the reset **~500 ms before the next GNSS pulse**, offset ~6 ms
> earlier to cancel those two known latencies.
>
> A resync is triggered only when the measured GNSS↔RTC drift exceeds
> **10 ms** (`SYNC_THRESHOLD_US`); below that the RTC is left free-running. The
> alignment a resync achieves is not exact and converges over time: the first
> one can be off by up to **±500 µs**, settling to roughly **±100 µs** around
> zero in steady state, with occasional **±200 µs** excursions.

## Hardware

Pin, timer and bus assignments are defined in
`src/platform/maxim/parameters.h`:

| Define                                       | Value                | Description                                |
| -------------------------------------------- | -------------------- | ------------------------------------------ |
| `GNSS_UART_DEVICE_ID`                        | 2                    | UART2 to the u-blox module                 |
| `GNSS_RESET_PORT` / `GNSS_RESET_PIN`         | 2 / 7                | GNSS reset line                            |
| `GNSS_PPS_IRQ_PORT` / `GNSS_PPS_IRQ_PIN`     | 0 / 6                | GNSS 1PPS interrupt (`GPIO0_IRQn`)         |
| `RTC_I2C_DEVICE_ID` / `RTC_I2C_BAUDRATE`     | 1 / 400000           | MAX31343 on I2C1 @ 400 kHz                 |
| `RTC_PPS_IRQ_PORT` / `RTC_PPS_IRQ_PIN`       | 2 / 13               | RTC SQW 1PPS interrupt (`GPIO2_IRQn`)      |
| `PPS_TIMER_ID` / `PPS_TIMER_FREQ_HZ`         | 1 / 60000000         | Free-running capture timer (TMR1 @ 60 MHz) |
| `RTC_SYNC_TIMER_REGS` / `RTC_SYNC_TIMER_IRQ` | MXC_TMR2 / TMR2_IRQn | One-shot timer used during RTC realignment |

## Building

Enable the subsystem by adding `TIME_SYNC=y` to the build:

```bash
make reset
export PLATFORM=maxim
export TARGET=max32650
export INTERFACE=usb
make PQLIB_PATH=<path_to_library> TIME_SYNC=y run
```

Without `TIME_SYNC=y` the time-sync code is compiled out entirely and the
GNSS/RTC attributes are not exposed.

## IIO Attributes

When built with `TIME_SYNC=y`, the `pqm` device exposes these additional
read-only attributes:

| Attribute          | R/W | Description                                                     |
| ------------------ | --- | --------------------------------------------------------------- |
| `time_ms`          | R   | Live wall-clock time, advanced every 1PPS pulse (ms)            |
| `pps_drift_us`     | R   | Measured phase offset between the GNSS and RTC 1PPS pulses (µs) |
| `time_sync_status` | R   | `gnss_locked` / `holdover` / `unsynced` (see below)             |

Only **live** values are exposed. The GNSS calendar fields (year, month, day,
hour, minute, second, nanoseconds, Unix timestamp and time-accuracy) are read
from the receiver **once, during boot** — after that the UART to the GNSS
module is handed over to the upstream data path, so those fields would freeze
at their boot value. Exposing them over IIO would be misleading, so they are
intentionally omitted.

The live wall clock is instead carried by `time_ms`. It is **source-agnostic**:
seeded once at boot (from the GNSS fix, or from the RTC if no fix is present)
and then advanced by 1000 ms on every 1PPS edge — the GNSS pulse while locked,
the RTC pulse in holdover. So it keeps ticking with the correct value even
after GNSS is lost. It drives `pqm_update_timestamp()`; read `time_sync_status`
alongside it to know whether the current second came from GNSS or the RTC.

> **Note:** `pps_drift_us` reads `unavailable` until a valid GNSS fix has been
> acquired and the first synchronization has completed.

### Interpreting `time_sync_status` and `pps_drift_us`

`time_sync_status` is the **authoritative** indicator of whether the drift
value is meaningful:

| Status        | Meaning                                   | Trust `pps_drift_us`? |
| ------------- | ----------------------------------------- | --------------------- |
| `gnss_locked` | GNSS PPS is disciplining the clock (µs)   | Yes                   |
| `holdover`    | GNSS lost; RTC is free-running (drifting) | No — stale value      |
| `unsynced`    | No fix yet / boot not complete            | No                    |

`pps_drift_us` is defined as **`rtc_ts − gnss_ts`**: a positive value means the
RTC pulse arrives _after_ the GNSS pulse (RTC ahead). It is only recomputed
when both edges are fresh in the same second; in `holdover` it is not updated
and retains its last value.

## Known limitation: `time_ms` under a long loop stall

`time_ms` advances by exactly 1000 ms per processed 1PPS edge, driven by the
main-loop state machine. The per-edge flags it consumes are boolean, so if the
main loop is blocked for **longer than one second** — for example while a large
IIO `READBUF` transfer drains over the serial link — two 1PPS edges collapse
into one and the state machine advances `time_ms` only once. The clock then
stays **one second behind** and does not self-correct while it is running on
the RTC (holdover), because the run-time path never re-anchors to an absolute
epoch the way the boot path does.

The sub-second cycle timestamp is unaffected — it is re-anchored on every 1PPS
edge and folds any gap back into a single second — so this shows up only as a
whole-second offset on the wall clock, and only after a stall longer than one
second.

**Recovery:** the error is cleared by re-seeding the timebase from a fresh GNSS
fix at run time. Once GNSS is re-acquired, reading the absolute epoch from the
receiver and re-anchoring `time_ms` removes any accumulated whole-second offset.
The free-running holdover clock does not correct this on its own.

## Known limitation: UART baud-rate scan on the no-OS driver

This limitation applies to the **no-OS UART** path.

`gnss_init_set_baud()` auto-detects the GNSS module's baud rate by opening and
closing the UART once per candidate rate (`no_os_uart_init` / `no_os_uart_remove`
in a loop), with `asynchronous_rx = true` so an empty RX FIFO returns `-EAGAIN`
instead of blocking.

With the no-OS MAX32650 driver this scan only works on the **first** candidate
after a cold start. `no_os_uart_init` with `asynchronous_rx` arms an async RX
transaction in the MSDK (setting `RxAsyncRequests[uart_num]`), but
`max_uart_remove` tears down only the software FIFO and calls
`MXC_UART_Shutdown` — it never aborts the async transaction. The MSDK slot
stays non-NULL, so the next `no_os_uart_init` hits an `E_BAD_STATE` guard in
`MXC_UART_RevA_TransactionAsync` and never re-arms RX capture. Every candidate
after the first then reads only `-EAGAIN`, and detection reports
`no baudrate worked` even when the module is physically present.

On no-OS the clean fix belongs in the platform driver — `max_uart_remove` should
abort the RX transaction (via `MXC_UART_RxAbortAsync`, the RX-specific call;
`MXC_UART_AbortAsync` returns early on RX-only transactions and leaves the slot
armed) before shutdown. It is not applied here, so on the current driver the
scan is reliable only on a first-boot detection where the target baud is the
first candidate tried.

## Reading the attributes over IIO

The device and its attributes are enumerated with `iio_info`:

```bash
iio_info -u serial:COM34,115200,8n1
```

Each time-sync attribute is read with `iio_attr`:

```bash
iio_attr -u serial:COM34,115200,8n1 -d pqm time_sync_status
iio_attr -u serial:COM34,115200,8n1 -d pqm time_ms
iio_attr -u serial:COM34,115200,8n1 -d pqm pps_drift_us
```

`time_sync_status` reports `gnss_locked` once boot completes and a GNSS fix is
disciplining the clock, `holdover` after GNSS is lost, and `unsynced` before the
first sync. `time_ms` carries the live wall clock, and `pps_drift_us` reports the
GNSS↔RTC phase offset — meaningful only while `gnss_locked`, close to zero when
the RTC 1PPS is well aligned to the GNSS pulse.
