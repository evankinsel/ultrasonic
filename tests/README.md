# Bench validation checklist

- [ ] Compiles for `arduino:renesas_uno:unor4wifi` in Arduino IDE/CLI.
- [ ] D9 emits ~10 us trigger pulses at least 60 ms apart.
- [ ] With fixed target, D2 ECHO has a plausible width and `OK` status.
- [ ] With no echo, emits `NO_RISE` or `NO_FALL` within ~30 ms.
- [ ] 50 `DATA` records follow `BURST 50`; `STOP` clears queued trials.
- [ ] Verify interrupt-based echo timing and latency against oscilloscope.
- [ ] With safe calibrated input, test A0 14-bit scaling, bias, clipping and noise.
- [ ] Validate analog sampling window / hold-time response for expected echo distances.
- [ ] Check `micros()` rollover and long-running 24-hour serial logging on a host.
