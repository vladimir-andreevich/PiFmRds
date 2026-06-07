# Debugging RF Startup

This branch adds controlled runtime diagnostics for unstable RF startup. Normal
runs are unchanged; diagnostics are only printed when `-debug` is present.

Build and run as usual:

```sh
cd src
make
sudo ./pi_fm_rds -freq 107.9 -audio sound.wav -debug 2>rf_startup.log
```

The debug output goes to `stderr`. Existing normal status output still goes to
`stdout`, so redirect `stderr` when you want a clean diagnostic log.

Key signals in the log:

* Startup register dumps show DMA, PWM, GPCLK and PWMCLK state around mmap,
  GPIO4 ALT0 setup, GPCLK carrier start, DMA control block build, PWM setup,
  DMA start, and `baseband_open()`.
* The startup probe samples registers for about 150 ms after DMA start. Watch
  `dma_conblk_ad`: it should move through the DMA ring.
* Periodic loop dumps are rate-limited to about once per second and include
  `loop_count`, DMA/PWM/clock registers, current/last sample positions,
  `free_slots`, `written_samples`, DMA stall count and `intval` stats.

Interpretation guide:

* If bad mode shows `dma_conblk_ad` not changing, investigate PWM pacing, DMA
  start, DREQ, PWM clock setup and reset order.
* If `dma_conblk_ad` moves, `written_samples` grows and `intval_min/max` are
  nonzero, investigate the GPCLK divisor update path, clock sequencing,
  physical memory mapping and cache coherency.
* If `intval_min == 0`, `intval_max == 0` and `intval_nonzero_count == 0`,
  investigate WAV input, baseband generation and scaling.
* If bad mode disappears after delays or reset-order changes, focus on startup
  sequencing and cleanup. The current cleanup logs GPCLK stop and DMA reset;
  PWM/PWMCLK stop is intentionally left as a follow-up experiment.
