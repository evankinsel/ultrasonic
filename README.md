# UltrasonicDAQ | UNO R4 WiFi

Embedded C++ foundation for the **Development and Performance Characterization of a Low-Cost Ultrasonic Instrumentation Rig for Free-Surface Reflection Mapping** (Project Draft 5, 2 Nov 2026).

**Status:** firmware v0.1, intended for bench validation; host-side syntax and simulated 50-trigger smoke test completed, but not yet compiled against a physical UNO R4 WiFi or verified on hardware. The primary result is characterization of sensor timing, analog echo detectability and uncertainty, not direct density measurement.

## Hardware and pin assignments

| UNO R4 WiFi | HC-SR04 / other hardware | Purpose |
| --- | --- | --- |
| D9 | TRIG | 10 us acoustic burst trigger |
| D2 | ECHO | Interrupt-based pulse edges with `micros()` timestamps |
| A0 | Conditioned amplified receiver tap **only after verification** | Optional 14-bit echo-envelope peak-proxy acquisition |
| 5V | VCC | 5 V sensor supply |
| GND | GND | Common ground |

**DO NOT connect a bare sensor transducer or unknown HC-SR04 internal node directly to A0.** Verify the board revision and tap waveform on an oscilloscope first, including DC bias, maximum/minimum voltage, impedance, gain and polarity. Confirm A0 always stays within its electrical input limits. An amplifier/buffer or AC coupling may be necessary. The proposed BAT42 + 10 kΩ + 10 nF detector is *experimental*; its 100 us discharge time constant does not guarantee peak capture. Keep water isolated from all electronics.

## Install

1. Install the **Arduino IDE** (2.x) and the **Arduino UNO R4 Boards** package (Renesas core).
2. Open `firmware/UltrasonicDAQ/UltrasonicDAQ.ino` and select **Arduino UNO R4 WiFi**.
3. Wire **D9, D2, 5 V and GND only** for the first test. Use a fixed reflective target, such as a flat board.
4. Upload the sketch. Open the Serial Monitor at **115200 baud**, with line ending set to **Newline**.
5. Send `PING`, then `BURST 50`, then `STATUS` to test the timing path.
6. Leave analog capture **OFF** until the conditioned A0 input has been safely validated on a scope. To enable it afterward, send `ANALOG ON`.

## Serial protocol

Commands are line-oriented ASCII:

| Command | Effect |
| --- | --- |
| `PING` | Queue one trigger |
| `BURST 50` | Queue 50 triggers; at least 60 ms between trigger starts |
| `STOP` | Clear queued triggers (in-flight measurement still completes) |
| `STATUS` | Print board mode, analog capture state and queue status |
| `ANALOG ON` | Enable A0 envelope acquisition; only when idle and after circuit validation |
| `ANALOG OFF` | Disable A0 acquisition; only when idle |
| `HELP` | List commands |

Comments, acknowledgments and errors begin with `#`. Measurement records are CSV rows beginning with `DATA`:

```text
# DATA,trial,device_ms,trigger_us,rise_us,fall_us,echo_us,adc_peak,adc_peak_offset_us,adc_samples,status
DATA,1,7501,7499999,7500345,7501220,875,-1,-1,0,OK
```

The line above is an **illustrative schema example, not a measurement**. A field of `-1` means that data are unavailable (for example analog capture was off). `device_ms` is the device's approximate milliseconds-since-boot at the trigger; `trigger_us`, `rise_us`, and `fall_us` are `micros()` values that roll over about every 71.6 minutes. `echo_us` is the rising-to-falling ECHO pulse width, *not* a validated acoustic arrival timestamp. Add a host timestamp in future Python logging. Do not interpret an `OK` label as proof of correct distance or amplitude.

The analog ADC peak is a **raw ADC code**, not volts or pressure. The selected sampling window is from 300 us after triggering until 800 us after the ECHO falling edge (with a 30 ms timeout); it may capture other reflections, trigger feedthrough or baseline bias. The echo packet can arrive near the ECHO pulse's falling edge, and this window and detector RC time constant require oscilloscope validation. More ADC bits do not by themselves imply more effective accuracy.

## Project milestones

- **A: Digital timing:** establish stable ranging against a fixed reference with `PING` and `BURST 50`; assess false echoes, timeouts, interrupt latency and jitter.
- **B: Receiver tap:** inspect the actual HC-SR04 revision's amplified receiver node with a high-impedance scope probe.
- **C: Analog conditioning:** test BAT42 / RC detector against the original waveform, then enable A0 and establish peak capture reliability.
- **D: Python DAQ:** add a serial acquisition script that schedules 50 triggers every 10 minutes for 24 hours (7,200 scheduled readings), saves raw CSV and performs analysis.
- **E: Optional extensions:** Wi-Fi telemetry, improved hardware capture and custom signal-conditioning PCB in KiCad.

No AVR Timer1 registers or `ICP1` capture mode are used: the UNO R4 WiFi uses a **Renesas RA4M1**. `micros()` in an interrupt is a software timestamp, not 62.5 ns hardware capture. Validate jitter and effective resolution experimentally. Internal ADC reference voltage and total analog noise must be calibrated before interpreting codes as voltage.

## References

- [Arduino UNO R4 WiFi hardware documentation](https://docs.arduino.cc/hardware/uno-r4-wifi)
- [Arduino attachInterrupt reference](https://docs.arduino.cc/language-reference/en/functions/external-interrupts/attachInterrupt/)
- [Arduino analogReadResolution reference](https://docs.arduino.cc/language-reference/en/functions/analog-io/analogReadResolution/)

This repository begins with C++ firmware. Python automation is intentionally deferred to the next implementation stage.
