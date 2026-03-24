/*
  Automatic Resin Dispensing - Arduino Nano + DM806i (STEP/DIR/ENA)

  WIRING (matches schematic):
    D2 -> PUL+ (STEP)
    D4 -> DIR+ (DIR)
    D7 -> ENA+ (ENABLE)
    GND -> PUL-, DIR-, ENA-

  TRIGGER:
    D8 -> "DISPENSE_NOW" (HIGH = start). Current DAR diagram shows 12V->3.3V converter into D8.

  LIMIT SWITCHES (NO contacts, wired to GND):
    D3 -> fully extended end stop
    D5 -> fully retracted end stop (home position)

  E-STOP: wired directly to PSU / driver power path (hardware-only).
          No Arduino pin needed. Cutting power stops the driver immediately.

  STATUS LED:
    D13 -> built-in LED. IDLE=off, POURING=solid on, DONE=off, ABORTED=rapid blink.

  HOW IT WORKS:
    - Converts target mL to linear travel using syringe diameter
    - Converts travel to motor steps using leadscrew lead + microsteps
    - Supports interval pours: pour SEGMENT_ML, wait WAIT_BETWEEN_S, repeat
    - Optional anti-drip retract at end
    - Checks limit switches on every step pulse
    - Unexpected limit hit during dispense: driver disabled, system locked until RESET

  CHANGE THESE DEFAULTS to match hardware:
    - MICROSTEPS must match DM806i DIP switch setting
    - LEADSCREW_LEAD_MM (T8x2 => 2.0 mm lead)
    - SYRINGE_ID_MM (50.8 mm)
*/

#include <Arduino.h>

// ---------------- PINOUT ----------------
const uint8_t PIN_STEP      = 6;   // PUL+  (D6)
const uint8_t PIN_DIR       = 4;   // DIR+  (D4)
const uint8_t PIN_EN        = 7;   // ENA+  (D7)
const uint8_t PIN_TRIGGER   = 8;   // start signal from printer (not wired yet)
const uint8_t PIN_LIMIT_EXT = 10;  // D10 - fully extended end stop (not wired yet)
const uint8_t PIN_LIMIT_RET = 11;  // D11 - fully retracted / home (not wired yet)
const uint8_t PIN_LED       = 13;  // built-in LED - state indicator

// ---------------- MOTION / MECH PARAMS ----------------
const long MOTOR_FULL_STEPS_PER_REV = 200;
const long MICROSTEPS = 10;
const float LEADSCREW_LEAD_MM = 2.0f;

// Syringes: internal diameter (mm)
const float SYRINGE_ID_MM = 50.8f;

// Calibration factor 1.02 was used in last sem testing.
float CAL_FACTOR = 1.02f;

// If target volume is TOTAL mixed output (A+B), keep this true.
// If instead we want "per syringe" volume, set false.
const bool VOLUME_IS_TOTAL_MIXED = true;

// ---------------- DISPENSE SETTINGS ----------------
float TARGET_TOTAL_ML  = 100.0f;  // default target
float SEGMENT_ML       = 50.0f;   // pour in chunks (set = TARGET to pour all at once)
unsigned long WAIT_BETWEEN_S = 60; // wait between chunks (seconds)

// Speed control:
// We step at a constant rate derived from RPM.
// 30 rpm @ 3200 steps/rev => 1600 steps/s => 625 us period per step (approx).
float MOTOR_RPM = 8.0f;

// Common-anode wiring: LOW = current flows through optocoupler.
// DM806I: ENA active (current) = motor DISABLED. So enable = no current = HIGH.
const bool ENABLE_ACTIVE_HIGH = true;

// Pick which way is "dispense" vs "retract"
const bool DIR_DISPENSE_LEVEL = LOW;

// Anti-drip retract at end (mm). Set 0 to disable.
float RETRACT_MM = 0.5f;

// Debounce / trigger behavior
const unsigned long TRIGGER_DEBOUNCE_MS = 50;

const long STEPS_PER_REV = MOTOR_FULL_STEPS_PER_REV * MICROSTEPS;

// ---------------- STATE (PR-05) ----------------
enum DispenseState { STATE_IDLE, STATE_POURING, STATE_DONE, STATE_ABORTED, STATE_PAUSED };
DispenseState g_state = STATE_IDLE;
bool g_aborted = false;
volatile bool g_stopRequested = false;

// Pause/resume tracking — saves position mid-dispense
float g_pauseRemainingMl = 0.0f;
long  g_pauseRemainingSteps = 0;
bool  g_pauseInSegment = false;  // true if paused mid-segment (steps remain)

void setState(DispenseState s) {
  g_state = s;
  switch (s) {
    case STATE_IDLE:
      g_aborted = false;
      digitalWrite(PIN_LED, LOW);
      Serial.println(F("STATE: IDLE"));
      break;
    case STATE_POURING:
      digitalWrite(PIN_LED, HIGH);
      Serial.println(F("STATE: POURING"));
      break;
    case STATE_DONE:
      digitalWrite(PIN_LED, LOW);
      Serial.println(F("STATE: DONE - ready for cure"));
      break;
    case STATE_ABORTED:
      g_aborted = true;
      // LED rapid blink handled in loop()
      Serial.println(F("STATE: ABORTED - send RESET to resume"));
      break;
    case STATE_PAUSED:
      // LED slow blink handled in loop()
      Serial.println(F("STATE: PAUSED - send RESUME to continue or RESET to cancel"));
      break;
  }
}

// ---------------- MATH ----------------
float syringeArea_mm2() {
  float r = SYRINGE_ID_MM * 0.5f;
  return 3.1415926f * r * r;
}

// mL per mm of plunger travel (per syringe)
float mlPerMmPerSyringe() {
  // 1 mL = 1000 mm^3
  return syringeArea_mm2() / 1000.0f;
}

// Convert (total mixed mL) -> required plunger travel (mm)
float travelMmForTotalMl(float totalMl) {
  float perSyringeMl = VOLUME_IS_TOTAL_MIXED ? (totalMl * 0.5f) : totalMl;
  perSyringeMl *= CAL_FACTOR;
  return perSyringeMl / mlPerMmPerSyringe();
}

// Convert travel (mm) -> motor steps
long stepsForTravelMm(float travelMm) {
  float revs = travelMm / LEADSCREW_LEAD_MM;
  return (long)(revs * (float)STEPS_PER_REV + 0.5f);
}

// ---------------- MOTION ----------------
// Timing: step pulse width for DM806i (some units need 20+ us)
const unsigned int STEP_PULSE_US = 50;

// Compute step interval in microseconds from RPM
unsigned long stepIntervalUsFromRPM(float rpm) {
  float stepsPerSec = (rpm * (float)STEPS_PER_REV) / 60.0f;
  if (stepsPerSec < 1.0f) stepsPerSec = 1.0f;
  return (unsigned long)(1000000.0f / stepsPerSec);
}

void setEnable(bool enabled) {
  digitalWrite(PIN_EN, ENABLE_ACTIVE_HIGH ? (enabled ? HIGH : LOW) : (enabled ? LOW : HIGH));
}

void stepOnce(unsigned long stepIntervalUs) {
  digitalWrite(PIN_STEP, LOW);   // common-anode: LOW = active pulse
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(PIN_STEP, HIGH);  // common-anode: HIGH = idle
  if (stepIntervalUs > STEP_PULSE_US) {
    delayMicroseconds(stepIntervalUs - STEP_PULSE_US);
  }
}

// Check serial buffer for a STOP command without blocking.
// Reads available bytes looking for "STOP\n". Sets g_stopRequested if found.
void checkForStopCommand() {
  if (!Serial.available()) return;
  // Peek at incoming data — read line if available
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line == "STOP") {
    g_stopRequested = true;
  }
  // Other commands during motion are ignored (they'd conflict)
}

// Move steps in given direction.
// Checks limit switches on every step — UC-05-FR2: hard stop within 1 step.
// Checks for STOP command every 50 steps for pause support.
// An unexpected limit hit sets ABORTED state (requires RESET to resume).
// Returns the number of steps remaining (0 = all completed).
long moveSteps(long steps, bool dispenseDir) {
  // Common-anode: LOW = active. Invert DIR_DISPENSE_LEVEL for common-anode.
  digitalWrite(PIN_DIR, dispenseDir ? !DIR_DISPENSE_LEVEL : DIR_DISPENSE_LEVEL);
  unsigned long dt = stepIntervalUsFromRPM(MOTOR_RPM);

  for (long i = 0; i < steps; i++) {
    // Check for STOP every 50 steps (fast enough to respond, minimal overhead)
    if (i % 50 == 0) {
      checkForStopCommand();
      if (g_stopRequested) {
        return steps - i;  // return remaining steps
      }
    }
    // Extended limit: only checked when dispensing — UC-05-FR2
    if (dispenseDir && digitalRead(PIN_LIMIT_EXT) == LOW) {
      setEnable(false);
      setState(STATE_ABORTED);
      Serial.println(F("LIMIT: extend end-stop triggered. Send RESET to resume."));
      return -(steps - i);  // negative = aborted (not pauseable)
    }
    // Retract limit: only checked when retracting — UC-05-FR2
    if (!dispenseDir && digitalRead(PIN_LIMIT_RET) == LOW) {
      setEnable(false);
      setState(STATE_ABORTED);
      Serial.println(F("LIMIT: retract end-stop triggered. Send RESET to resume."));
      return -(steps - i);  // negative = aborted
    }
    stepOnce(dt);
  }
  return 0;  // all steps completed
}

// UC-05-FR3: Home by retracting until the retract limit switch activates.
void homeRetract() {
  if (digitalRead(PIN_LIMIT_RET) == LOW) {
    Serial.println(F("HOME: already at retract limit."));
    return;
  }
  Serial.println(F("HOME: retracting to limit switch..."));
  setEnable(true);
  // Retract direction is opposite of dispense (inverted for common-anode)
  digitalWrite(PIN_DIR, DIR_DISPENSE_LEVEL ? HIGH : LOW);
  unsigned long dt = stepIntervalUsFromRPM(MOTOR_RPM);

  while (digitalRead(PIN_LIMIT_RET) == HIGH) {
    stepOnce(dt);
  }
  setEnable(false);
  Serial.println(F("HOME: complete."));
}

// Retract syringes by a specified volume (mL).
// Pulls plungers back to relieve pressure or withdraw resin from the nozzle.
void retractMl(float ml) {
  if (g_aborted) { Serial.println(F("Aborted. Send RESET first.")); return; }
  Serial.print(F("RETRACT: ")); Serial.print(ml, 2); Serial.println(F(" mL"));
  float travelMm = travelMmForTotalMl(ml);
  long steps = stepsForTravelMm(travelMm);
  Serial.print(F(" travel mm: ")); Serial.print(travelMm, 3);
  Serial.print(F(" | steps: ")); Serial.println(steps);
  setEnable(true);
  moveSteps(steps, false);  // false = retract direction
  setEnable(false);
  if (!g_aborted) Serial.println(F("RETRACT: complete."));
}

// UC-06-FR2: Dispense mL to waste container then retract (purge/clean cycle).
void purge(float ml) {
  if (g_aborted) { Serial.println(F("Aborted. Send RESET first.")); return; }
  Serial.print(F("PURGE: dispensing ")); Serial.print(ml, 1); Serial.println(F(" mL to waste..."));
  long steps = stepsForTravelMm(travelMmForTotalMl(ml));
  setEnable(true);
  moveSteps(steps, true);
  if (!g_aborted && RETRACT_MM > 0.0f) {
    moveSteps(stepsForTravelMm(RETRACT_MM), false);
  }
  setEnable(false);
  if (!g_aborted) Serial.println(F("PURGE: complete."));
}

// Dispense total mL (possibly in chunks). Supports mid-dispense pause via STOP.
void dispenseTotalMl(float totalMl) {
  if (g_aborted) { Serial.println(F("Aborted. Send RESET first.")); return; }

  Serial.print(F("Dispense request (total mixed mL): "));
  Serial.println(totalMl, 3);
  setState(STATE_POURING);
  g_stopRequested = false;

  float remaining = totalMl;

  while (remaining > 0.0001f && !g_aborted && !g_stopRequested) {
    float thisSeg = SEGMENT_ML;
    if (thisSeg <= 0) thisSeg = remaining;
    if (thisSeg > remaining) thisSeg = remaining;

    float travelMm = travelMmForTotalMl(thisSeg);
    long steps = stepsForTravelMm(travelMm);

    Serial.print(F(" Segment mL: ")); Serial.print(thisSeg, 3);
    Serial.print(F(" | travel mm: ")); Serial.print(travelMm, 3);
    Serial.print(F(" | steps: ")); Serial.println(steps);

    setEnable(true);
    long stepsLeft = moveSteps(steps, true);
    setEnable(false);

    if (stepsLeft < 0) break;   // aborted (limit switch)

    if (stepsLeft > 0) {
      // Paused mid-segment — save state for RESUME
      g_pauseRemainingSteps = stepsLeft;
      g_pauseRemainingMl = remaining - thisSeg;  // mL after this segment
      g_pauseInSegment = true;
      setState(STATE_PAUSED);
      Serial.print(F("PAUSED: ")); Serial.print(stepsLeft);
      Serial.println(F(" steps remain in segment."));
      Serial.println(F("OK"));
      return;
    }

    remaining -= thisSeg;

    if (remaining > 0.0001f && WAIT_BETWEEN_S > 0) {
      Serial.print(F(" Waiting ")); Serial.print(WAIT_BETWEEN_S);
      Serial.println(F(" s before next segment..."));
      // Check for STOP during wait interval (check every 100ms)
      for (unsigned long w = 0; w < WAIT_BETWEEN_S * 10UL; w++) {
        delay(100);
        checkForStopCommand();
        if (g_stopRequested) {
          g_pauseRemainingSteps = 0;
          g_pauseRemainingMl = remaining;
          g_pauseInSegment = false;
          setState(STATE_PAUSED);
          Serial.print(F("PAUSED: ")); Serial.print(remaining, 3);
          Serial.println(F(" mL remaining."));
          Serial.println(F("OK"));
          return;
        }
      }
    }
  }

  if (!g_aborted && !g_stopRequested) {
    // Anti-drip retract
    if (RETRACT_MM > 0.0f) {
      long rSteps = stepsForTravelMm(RETRACT_MM);
      Serial.print(F("Retract mm: ")); Serial.print(RETRACT_MM, 3);
      Serial.print(F(" | steps: ")); Serial.println(rSteps);
      setEnable(true);
      moveSteps(rSteps, false);
      setEnable(false);
    }
    if (!g_aborted) {
      setState(STATE_DONE);
      Serial.println(F("DONE."));
    }
  }
}

// Resume a paused dispense from saved state.
void resumeDispense() {
  if (g_state != STATE_PAUSED) {
    Serial.println(F("Not paused — nothing to resume."));
    return;
  }

  Serial.println(F("Resuming dispense..."));
  setState(STATE_POURING);
  g_stopRequested = false;

  // Finish remaining steps in the interrupted segment
  if (g_pauseInSegment && g_pauseRemainingSteps > 0) {
    Serial.print(F(" Completing segment: ")); Serial.print(g_pauseRemainingSteps);
    Serial.println(F(" steps"));
    setEnable(true);
    long stepsLeft = moveSteps(g_pauseRemainingSteps, true);
    setEnable(false);

    if (stepsLeft < 0) return;  // aborted
    if (stepsLeft > 0) {
      // Paused again mid-segment
      g_pauseRemainingSteps = stepsLeft;
      setState(STATE_PAUSED);
      Serial.println(F("PAUSED again."));
      Serial.println(F("OK"));
      return;
    }
  }

  // Continue with remaining mL (full segments)
  if (g_pauseRemainingMl > 0.0001f) {
    dispenseTotalMl(g_pauseRemainingMl);
  } else if (!g_aborted && g_state != STATE_PAUSED) {
    // Anti-drip retract
    if (RETRACT_MM > 0.0f) {
      long rSteps = stepsForTravelMm(RETRACT_MM);
      setEnable(true);
      moveSteps(rSteps, false);
      setEnable(false);
    }
    if (!g_aborted) {
      setState(STATE_DONE);
      Serial.println(F("DONE."));
    }
  }
}

// ---------------- SERIAL COMMAND PARSER ----------------
// Commands:
//   V 150       -> set target total mL
//   SEG 50      -> set segment mL
//   WAIT 60     -> set wait seconds between segments
//   RPM 30      -> set motor RPM
//   CAL 1.02    -> set calibration factor
//   GO          -> start dispensing
//   HOME        -> retract to home (retract limit switch)
//   RETRACT 5   -> retract syringes by mL (pull plungers back)
//   PURGE 20    -> dispense mL to waste then retract
//   RESET       -> clear aborted state, return to IDLE
//   STATUS      -> print current settings and state
void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  auto startsWith = [&](const char* p) {
    return line.startsWith(String(p));
  };

  if (startsWith("V ")) {
    TARGET_TOTAL_ML = line.substring(2).toFloat();
  } else if (startsWith("SEG ")) {
    SEGMENT_ML = line.substring(4).toFloat();
  } else if (startsWith("WAIT ")) {
    WAIT_BETWEEN_S = (unsigned long)line.substring(5).toInt();
  } else if (startsWith("RPM ")) {
    MOTOR_RPM = line.substring(4).toFloat();
  } else if (startsWith("CAL ")) {
    CAL_FACTOR = line.substring(4).toFloat();
  } else if (startsWith("RETRACT ")) {
    retractMl(line.substring(8).toFloat());
  } else if (startsWith("PURGE ")) {
    purge(line.substring(6).toFloat());
  } else if (line == "GO") {
    dispenseTotalMl(TARGET_TOTAL_ML);
  } else if (line == "OVERRIDE") {
    Serial.println(F("OVERRIDE: manual dispense trigger."));
    dispenseTotalMl(TARGET_TOTAL_ML);
  } else if (line == "HOME") {
    homeRetract();
  } else if (line == "STOP") {
    // STOP during idle is a no-op (during motion it's handled in moveSteps)
    if (g_state == STATE_POURING) {
      g_stopRequested = true;  // will be picked up by moveSteps
    } else {
      Serial.println(F("Not pouring — nothing to stop."));
    }
  } else if (line == "RESUME") {
    resumeDispense();
  } else if (line == "RESET") {
    g_stopRequested = false;
    g_pauseRemainingSteps = 0;
    g_pauseRemainingMl = 0.0f;
    g_pauseInSegment = false;
    setState(STATE_IDLE);
    Serial.println(F("System reset. Ready."));
  } else if (line == "PING") {
    // Respond to connection test from GUI — no action needed
  } else if (line == "STATUS") {
    const char* stateNames[] = { "IDLE", "POURING", "DONE", "ABORTED", "PAUSED" };
    Serial.println(F("---- STATUS ----"));
    Serial.print(F("STATE="));             Serial.println(stateNames[g_state]);
    Serial.print(F("TARGET_TOTAL_ML="));   Serial.println(TARGET_TOTAL_ML, 3);
    Serial.print(F("SEGMENT_ML="));        Serial.println(SEGMENT_ML, 3);
    Serial.print(F("WAIT_BETWEEN_S="));    Serial.println(WAIT_BETWEEN_S);
    Serial.print(F("MOTOR_RPM="));         Serial.println(MOTOR_RPM, 3);
    Serial.print(F("CAL_FACTOR="));        Serial.println(CAL_FACTOR, 4);
    Serial.print(F("STEPS_PER_REV="));     Serial.println(STEPS_PER_REV);
    Serial.print(F("LEADSCREW_LEAD_MM=")); Serial.println(LEADSCREW_LEAD_MM, 3);
    Serial.print(F("SYRINGE_ID_MM="));     Serial.println(SYRINGE_ID_MM, 3);
    Serial.print(F("VOLUME_IS_TOTAL_MIXED=")); Serial.println(VOLUME_IS_TOTAL_MIXED ? "true" : "false");
    Serial.print(F("LIMIT_EXT(D3)="));     Serial.println(digitalRead(PIN_LIMIT_EXT) == LOW ? "TRIGGERED" : "open");
    Serial.print(F("LIMIT_RET(D5)="));     Serial.println(digitalRead(PIN_LIMIT_RET) == LOW ? "TRIGGERED" : "open");
    Serial.println(F("----------------"));
  } else {
    Serial.println(F("Unknown cmd. Try: STATUS, V <ml>, SEG <ml>, WAIT <s>, RPM <rpm>, CAL <x>, GO, HOME, PURGE <ml>, RETRACT <ml>, STOP, RESUME, RESET"));
  }

  Serial.println(F("OK"));
}

// Trigger detect (falling edge on D8 — pulled HIGH by INPUT_PULLUP, ground to trigger)
bool triggerStartDetected() {
  static bool last = true;
  bool now = digitalRead(PIN_TRIGGER);
  if (!now && last) {
    delay(TRIGGER_DEBOUNCE_MS);
    if (!digitalRead(PIN_TRIGGER)) {
      last = now;
      return true;
    }
  }
  last = now;
  return false;
}

void setup() {
  pinMode(PIN_STEP,      OUTPUT);
  pinMode(PIN_DIR,       OUTPUT);
  pinMode(PIN_EN,        OUTPUT);
  pinMode(PIN_TRIGGER,   INPUT_PULLUP);    // pulled HIGH when idle; ground D8 to trigger
  pinMode(PIN_LIMIT_EXT, INPUT_PULLUP);    // NO switch to GND: HIGH=open, LOW=triggered
  pinMode(PIN_LIMIT_RET, INPUT_PULLUP);    // NO switch to GND: HIGH=open, LOW=triggered
  pinMode(PIN_LED,       OUTPUT);

  digitalWrite(PIN_STEP, HIGH);  // common-anode: HIGH = idle (no current)
  digitalWrite(PIN_DIR,  HIGH);  // common-anode: HIGH = idle (no current)
  setEnable(false);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(115200);
  Serial.println(F("Resin Dispenser Ready."));
  Serial.println(F("Commands: STATUS, V <ml>, SEG <ml>, WAIT <s>, RPM <rpm>, CAL <x>, GO, HOME, PURGE <ml>, RETRACT <ml>, RESET"));
  setState(STATE_IDLE);
}

void loop() {
  handleSerial();

  // When aborted: blink LED rapidly and ignore all triggers (UC-04-FR3)
  if (g_aborted) {
    static unsigned long lastBlink = 0;
    static bool ledOn = false;
    if (millis() - lastBlink > 200) {
      ledOn = !ledOn;
      digitalWrite(PIN_LED, ledOn ? HIGH : LOW);
      lastBlink = millis();
    }
    return;
  }

  // When paused: blink LED slowly (1s on, 1s off)
  if (g_state == STATE_PAUSED) {
    static unsigned long lastPauseBlink = 0;
    static bool pauseLedOn = false;
    if (millis() - lastPauseBlink > 1000) {
      pauseLedOn = !pauseLedOn;
      digitalWrite(PIN_LED, pauseLedOn ? HIGH : LOW);
      lastPauseBlink = millis();
    }
    return;
  }

  if (triggerStartDetected()) {
    Serial.println(F("TRIGGER received on D8 -> starting dispense..."));
    dispenseTotalMl(TARGET_TOTAL_ML);
  }
}
