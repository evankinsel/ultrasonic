/*
 * UltrasonicDAQ v0.1 - UNO R4 WiFi
 *
 * Draft 5 measurement architecture:
 *   D9 -> ultrasonic TRIG
 *   D2 <- ultrasonic ECHO (CHANGE interrupt, software edge timestamps)
 *   A0 <- OPTIONAL conditioned analog envelope (14-bit ADC mode)
 *
 * Serial commands (115200 baud, newline):
 *   PING       one measurement
 *   BURST 50   queue 50 measurements, at least 60 ms apart
 *   ANALOG ON  enable A0 sampling (only AFTER hardware validation)
 *   ANALOG OFF disable A0 sampling
 *   STATUS     report configuration and progress
 *   STOP       clear pending measurements (current ping completes)
 *   HELP       show commands
 *
 * A DATA record is one trigger, not a chemical preparation or independent
 * experimental replicate. Host timestamps should be added in Python.
 *
 * IMPORTANT: micros() plus GPIO interrupts is NOT hardware input capture;
 * characterize interrupt latency, timestamp quantization, and sensor jitter.
 */

#include "config.h"
#include <ctype.h>
#include <stdint.h>
#include <string.h>

namespace {

enum class RunState : uint8_t { Idle, Listening, PostFall };

// ISR-owned fields. Take a synchronized snapshot in the main loop.
volatile bool irqArmed = false;
volatile bool irqSawRise = false;
volatile bool irqSawFall = false;
volatile uint32_t irqRiseUs = 0;
volatile uint32_t irqFallUs = 0;

RunState runState = RunState::Idle;
bool analogEnabled = config::ANALOG_ENABLED_AT_BOOT;
bool haveTriggered = false;
uint16_t queuedPings = 0;
uint32_t trialNumber = 0;
uint32_t lastTriggerUs = 0;
uint32_t activeTriggerUs = 0;
uint32_t activeHostUptimeMs = 0;
uint16_t peakAdc = 0;
int32_t peakOffsetUs = -1;
uint32_t adcSamples = 0;

char commandBuffer[48] = {};
uint8_t commandLength = 0;
bool discardCommand = false;

struct EdgeSnapshot {
  bool rise;
  bool fall;
  uint32_t riseUs;
  uint32_t fallUs;
};

// Unsigned subtraction correctly handles the ~71-minute micros() rollover
// when the interval being compared is shorter than 2^31 microseconds.
uint32_t elapsedUs(uint32_t now, uint32_t earlier) {
  return static_cast<uint32_t>(now - earlier);
}

EdgeSnapshot getEdges() {
  noInterrupts();
  EdgeSnapshot result{irqSawRise, irqSawFall, irqRiseUs, irqFallUs};
  interrupts();
  return result;
}

void echoEdgeISR() {
  if (!irqArmed) return;
  const uint32_t atUs = micros();
  if (digitalRead(config::ECHO_PIN) == HIGH) {
    if (!irqSawRise) {
      irqRiseUs = atUs;
      irqSawRise = true;
    }
  } else if (irqSawRise && !irqSawFall) {
    irqFallUs = atUs;
    irqSawFall = true;
  }
}

void beginPing() {
  // Reset ISR state before emitting a fresh trigger.
  noInterrupts();
  irqArmed = false;
  irqSawRise = false;
  irqSawFall = false;
  irqRiseUs = 0;
  irqFallUs = 0;
  interrupts();

  peakAdc = 0;
  peakOffsetUs = -1;
  adcSamples = 0;
  ++trialNumber;
  --queuedPings;

  // This short 10 us trigger is intentionally synchronous. Echo acquisition
  // is interrupt-based and the subsequent wait is nonblocking.
  digitalWrite(config::TRIG_PIN, LOW);
  delayMicroseconds(2);
  activeTriggerUs = micros();
  activeHostUptimeMs = millis();
  lastTriggerUs = activeTriggerUs;
  noInterrupts();
  irqArmed = true;
  interrupts();
  digitalWrite(config::TRIG_PIN, HIGH);
  delayMicroseconds(config::TRIGGER_PULSE_US);
  digitalWrite(config::TRIG_PIN, LOW);
  haveTriggered = true;
  runState = RunState::Listening;
}

void sampleEnvelope(uint32_t nowUs, const EdgeSnapshot& edges) {
  if (!analogEnabled) return;
  const uint32_t offset = elapsedUs(nowUs, activeTriggerUs);
  if (offset < config::ANALOG_BLANKING_US ||
      offset > config::ECHO_TIMEOUT_US + config::ANALOG_POST_FALL_US) return;
  if (edges.fall && elapsedUs(nowUs, edges.fallUs) > config::ANALOG_POST_FALL_US) return;

  // A0 provides an echo-related peak *proxy*, not a calibrated pressure value.
  // Sampling window and RC decay must be validated on a scope.
  const uint16_t code = static_cast<uint16_t>(analogRead(config::ENVELOPE_PIN));
  ++adcSamples;
  if (peakOffsetUs < 0 || code > peakAdc) {
    peakAdc = code;
    peakOffsetUs = static_cast<int32_t>(offset);
  }
}

void printRecord(const EdgeSnapshot& edges, const char* status) {
  // CSV line schema:
  // DATA,trial,device_ms,trigger_us,rise_us,fall_us,echo_us,
  //      adc_peak,adc_peak_offset_us,adc_samples,status
  Serial.print(F("DATA,"));
  Serial.print(trialNumber);
  Serial.print(',');
  Serial.print(activeHostUptimeMs);
  Serial.print(',');
  Serial.print(activeTriggerUs);
  Serial.print(',');
  if (edges.rise) Serial.print(edges.riseUs); else Serial.print(-1);
  Serial.print(',');
  if (edges.fall) Serial.print(edges.fallUs); else Serial.print(-1);
  Serial.print(',');
  if (edges.rise && edges.fall) {
    Serial.print(elapsedUs(edges.fallUs, edges.riseUs));
  } else {
    Serial.print(-1);
  }
  Serial.print(',');
  if (analogEnabled && adcSamples > 0) Serial.print(peakAdc); else Serial.print(-1);
  Serial.print(',');
  Serial.print(peakOffsetUs);
  Serial.print(',');
  Serial.print(adcSamples);
  Serial.print(',');
  Serial.println(status);
}

void finishPing(const EdgeSnapshot& edges, const char* status) {
  noInterrupts();
  irqArmed = false;
  interrupts();
  runState = RunState::Idle;
  printRecord(edges, status);
}

void processAcquisition() {
  if (runState == RunState::Idle) return;

  const uint32_t nowUs = micros();
  EdgeSnapshot edges = getEdges();
  sampleEnvelope(nowUs, edges);

  if (edges.fall) {
    runState = RunState::PostFall;
    if (!analogEnabled ||
        elapsedUs(micros(), edges.fallUs) >= config::ANALOG_POST_FALL_US) {
      const uint32_t widthUs = elapsedUs(edges.fallUs, edges.riseUs);
      finishPing(edges, (widthUs < 100 || widthUs > config::ECHO_TIMEOUT_US)
                            ? "BAD_PULSE" : "OK");
    }
    return;
  }

  if (elapsedUs(nowUs, activeTriggerUs) >= config::ECHO_TIMEOUT_US) {
    finishPing(edges, edges.rise ? "NO_FALL" : "NO_RISE");
  }
}

bool parseBurstCount(const char* arg, uint16_t& count) {
  if (!arg || !*arg) return false;
  uint32_t value = 0;
  for (const char* p = arg; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    value = value * 10UL + static_cast<uint32_t>(*p - '0');
    if (value > config::MAX_BURST_COUNT) return false;
  }
  if (value == 0) return false;
  count = static_cast<uint16_t>(value);
  return true;
}

void uppercase(char* value) {
  while (*value) {
    *value = static_cast<char>(toupper(static_cast<unsigned char>(*value)));
    ++value;
  }
}

void processCommand(char* line) {
  char* cmd = strtok(line, " \t");
  char* arg = strtok(nullptr, " \t");
  char* extra = strtok(nullptr, " \t");
  if (!cmd) return;
  uppercase(cmd);
  if (arg) uppercase(arg);

  if (!strcmp(cmd, "PING") && !arg) {
    if (queuedPings == config::MAX_BURST_COUNT) {
      Serial.println(F("# ERR queue_full"));
    } else {
      ++queuedPings;
      Serial.println(F("# OK queued 1"));
    }
  } else if (!strcmp(cmd, "BURST") && arg && !extra) {
    uint16_t requested = 0;
    if (!parseBurstCount(arg, requested) ||
        static_cast<uint32_t>(queuedPings) + requested > config::MAX_BURST_COUNT) {
      Serial.println(F("# ERR burst_count_must_be_1_to_1000_and_fit_queue"));
    } else {
      queuedPings += requested;
      Serial.print(F("# OK queued "));
      Serial.println(requested);
    }
  } else if (!strcmp(cmd, "ANALOG") && arg && !extra) {
    if (runState != RunState::Idle) {
      Serial.println(F("# ERR analog_mode_requires_idle"));
    } else if (!strcmp(arg, "ON")) {
      analogEnabled = true;
      Serial.println(F("# OK analog_on WARNING verify_A0_before_connecting"));
    } else if (!strcmp(arg, "OFF")) {
      analogEnabled = false;
      Serial.println(F("# OK analog_off"));
    } else {
      Serial.println(F("# ERR use_ANALOG_ON_or_ANALOG_OFF"));
    }
  } else if (!strcmp(cmd, "STOP") && !arg) {
    queuedPings = 0;
    Serial.println(F("# OK queue_cleared_current_ping_finishes"));
  } else if (!strcmp(cmd, "STATUS") && !arg) {
    Serial.print(F("# STATUS board=UNO_R4_WiFi adc_bits=14 analog="));
    Serial.print(analogEnabled ? F("ON") : F("OFF"));
    Serial.print(F(" state="));
    Serial.print(runState == RunState::Idle ? F("IDLE") : F("MEASURING"));
    Serial.print(F(" queued="));
    Serial.print(queuedPings);
    Serial.print(F(" started="));
    Serial.println(trialNumber);
  } else if (!strcmp(cmd, "HELP") && !arg) {
    Serial.println(F("# COMMANDS PING | BURST <1..1000> | ANALOG ON | ANALOG OFF | STATUS | STOP | HELP"));
  } else {
    Serial.println(F("# ERR unknown_command_or_arguments (try HELP)"));
  }
}

void pollSerial() {
  // Bounded processing time: one incoming line per loop iteration.
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      if (discardCommand) {
        Serial.println(F("# ERR command_too_long"));
      } else {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
      }
      discardCommand = false;
      commandLength = 0;
      return;
    }
    if (discardCommand) continue;
    if (static_cast<size_t>(commandLength) + 1 >= sizeof(commandBuffer)) {
      discardCommand = true;
    } else {
      commandBuffer[commandLength++] = c;
    }
  }
}

}  // namespace

void setup() {
  pinMode(config::TRIG_PIN, OUTPUT);
  digitalWrite(config::TRIG_PIN, LOW);
  pinMode(config::ECHO_PIN, INPUT);
  pinMode(config::ENVELOPE_PIN, INPUT);
  analogReadResolution(config::ADC_BITS);
  attachInterrupt(digitalPinToInterrupt(config::ECHO_PIN), echoEdgeISR, CHANGE);

  Serial.begin(config::SERIAL_BAUD);
  // Do not block boot waiting for a USB terminal; permits unattended logging.
  Serial.println(F("# UltrasonicDAQ v0.1 UNO_R4_WiFi"));
  Serial.println(F("# DATA,trial,device_ms,trigger_us,rise_us,fall_us,echo_us,adc_peak,adc_peak_offset_us,adc_samples,status"));
  Serial.println(F("# Analog OFF at boot. Send HELP for commands."));
}

void loop() {
  // Prefer sampling / edge handling before command parsing.
  processAcquisition();
  pollSerial();

  if (runState == RunState::Idle && queuedPings > 0) {
    const uint32_t nowUs = micros();
    if (!haveTriggered || elapsedUs(nowUs, lastTriggerUs) >= config::MIN_PING_INTERVAL_US) {
      beginPing();
    }
  }
}
