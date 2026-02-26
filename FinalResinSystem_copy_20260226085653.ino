/*
  Automatic Resin Dispensing - Arduino Uno + DM806i (STEP/DIR/ENA)

  WIRING (matches schematic):
    D2 -> PUL+ (STEP)
    D4 -> DIR+ (DIR)
    D7 -> ENA+ (ENABLE)
    GND -> PUL-, DIR-, ENA-

  TRIGGER:
    D8 -> "DISPENSE_NOW" (HIGH = start). Current DAR diagram shows 12V->3.3V converter into D8.

  HOW IT WORKS:
    - Converts target mL to linear travel using syringe diameter
    - Converts travel to motor steps using leadscrew lead + microsteps
    - Supports interval pours: pour SEGMENT_ML, wait WAIT_BETWEEN_S, repeat
    - Optional anti-drip retract at end

  CHANGE THESE DEFAULTS to match hardware:
    - MICROSTEPS must match DM806i DIP switch setting
    - LEADSCREW_LEAD_MM (T8x2 => 2.0 mm lead)
    - SYRINGE_ID_MM (50.8 mm)
*/

#include <Arduino.h>

// ---------------- PINOUT ----------------
const uint8_t PIN_STEP = 6;   // PUL+
const uint8_t PIN_DIR = 4;   // DIR+
const uint8_t PIN_EN = 7;   // ENA+
const uint8_t PIN_TRIGGER= 8;   // start signal from printer (HIGH = start)

// ---------------- MOTION / MECH PARAMS ----------------
const long MOTOR_FULL_STEPS_PER_REV = 200;
const long MICROSTEPS = 10;
const float LEADSCREW_LEAD_MM = 2.0f;  // T8x2 lead = 2.0 mm per revolution

// Syringes: internal diameter (mm)
const float  SYRINGE_ID_MM = 50.8f;

// Calibration factor 1.02 was used in last sem testing.
float CAL_FACTOR = 1.02f;

// If target volume is TOTAL mixed output (A+B), keep this true.
// If instead we want "per syringe" volume, set false.
const bool VOLUME_IS_TOTAL_MIXED = true;

// ---------------- DISPENSE SETTINGS ----------------
float TARGET_TOTAL_ML = 100.0f;  // default target
float SEGMENT_ML = 50.0f;   // pour in chunks (set = TARGET to pour all at once)
unsigned long WAIT_BETWEEN_S = 60; // wait between chunks (seconds)

// Speed control:
// We step at a constant rate derived from RPM.
// 30 rpm @ 3200 steps/rev => 1600 steps/s => 625 us period per step (approx).
float MOTOR_RPM = 30.0f;

// Enable polarity (depends on driver). Many drivers enable when EN is HIGH.
// If opposite, flip this.
const bool ENABLE_ACTIVE_HIGH = true;

// pick which way is "dispense" vs "retract"
const bool DIR_DISPENSE_LEVEL = HIGH;

// Anti-drip retract at end (mm). Set 0 to disable.
float RETRACT_MM = 0.5f;

// Debounce / trigger behavior
const unsigned long TRIGGER_DEBOUNCE_MS = 50;

const long STEPS_PER_REV = MOTOR_FULL_STEPS_PER_REV * MICROSTEPS;

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
  // apply calibration
  perSyringeMl *= CAL_FACTOR;

  float mm = perSyringeMl / mlPerMmPerSyringe();
  return mm;
}

// Convert travel (mm) -> motor steps
long stepsForTravelMm(float travelMm) {
  float revs = travelMm / LEADSCREW_LEAD_MM;
  long steps = (long) llround(revs * (float)STEPS_PER_REV);
  return steps;
}

// Timing: step pulse width for DM806i (safe > 5 us)
const unsigned int STEP_PULSE_US = 8;

// Compute step interval in microseconds from RPM
unsigned long stepIntervalUsFromRPM(float rpm) {
  float stepsPerSec = (rpm * (float)STEPS_PER_REV) / 60.0f;
  if (stepsPerSec < 1.0f) stepsPerSec = 1.0f;
  return (unsigned long)(1000000.0f / stepsPerSec);
}

void setEnable(bool enabled) {
  if (ENABLE_ACTIVE_HIGH) {
    digitalWrite(PIN_EN, enabled ? HIGH : LOW);
  } else {
    digitalWrite(PIN_EN, enabled ? LOW : HIGH);
  }
}

void stepOnce(unsigned long stepIntervalUs) {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(PIN_STEP, LOW);

  // remainder of interval
  if (stepIntervalUs > STEP_PULSE_US) {
    delayMicroseconds(stepIntervalUs - STEP_PULSE_US);
  }
}

void moveSteps(long steps, bool dispenseDir) {
  digitalWrite(PIN_DIR, dispenseDir ? DIR_DISPENSE_LEVEL : !DIR_DISPENSE_LEVEL);

  unsigned long dt = stepIntervalUsFromRPM(MOTOR_RPM);

  for (long i = 0; i < steps; i++) {
    stepOnce(dt);
  }
}

// Dispense total mL (possibly in chunks)
void dispenseTotalMl(float totalMl) {
  Serial.print(F("Dispense request (total mixed mL): "));
  Serial.println(totalMl, 3);

  float remaining = totalMl;

  while (remaining > 0.0001f) {
    float thisSeg = SEGMENT_ML;
    if (thisSeg <= 0) thisSeg = remaining;
    if (thisSeg > remaining) thisSeg = remaining;

    float travelMm = travelMmForTotalMl(thisSeg);
    long steps = stepsForTravelMm(travelMm);

    Serial.print(F(" Segment mL: ")); Serial.print(thisSeg, 3);
    Serial.print(F(" | travel mm: ")); Serial.print(travelMm, 3);
    Serial.print(F(" | steps: ")); Serial.println(steps);

    setEnable(true);
    moveSteps(steps, /*dispenseDir=*/true);
    setEnable(false);

    remaining -= thisSeg;

    if (remaining > 0.0001f && WAIT_BETWEEN_S > 0) {
      Serial.print(F(" Waiting ")); Serial.print(WAIT_BETWEEN_S);
      Serial.println(F(" s before next segment..."));
      delay(WAIT_BETWEEN_S * 1000UL);
    }
  }

  // Anti-drip retract
  if (RETRACT_MM > 0.0f) {
    long rSteps = stepsForTravelMm(RETRACT_MM);
    Serial.print(F("Retract mm: ")); Serial.print(RETRACT_MM, 3);
    Serial.print(F(" | steps: ")); Serial.println(rSteps);

    setEnable(true);
    moveSteps(rSteps, /*dispenseDir=*/false);
    setEnable(false);
  }

  Serial.println(F("DONE."));
}

// Simple serial command parser
// Commands (examples):
//   V 150        -> set target total mL
//   SEG 50       -> set segment mL
//   WAIT 60      -> set wait seconds between segments
//   RPM 30       -> set motor RPM
//   CAL 1.02     -> set calibration factor
//   GO           -> start dispensing
//   STATUS       -> print settings
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
    WAIT_BETWEEN_S = (unsigned long) line.substring(5).toInt();
  } else if (startsWith("RPM ")) {
    MOTOR_RPM = line.substring(4).toFloat();
  } else if (startsWith("CAL ")) {
    CAL_FACTOR = line.substring(4).toFloat();
  } else if (line == "GO") {
    dispenseTotalMl(TARGET_TOTAL_ML);
  } else if (line == "STATUS") {
    Serial.println(F("---- STATUS ----"));
    Serial.print(F("TARGET_TOTAL_ML=")); Serial.println(TARGET_TOTAL_ML, 3);
    Serial.print(F("SEGMENT_ML="));      Serial.println(SEGMENT_ML, 3);
    Serial.print(F("WAIT_BETWEEN_S="));  Serial.println(WAIT_BETWEEN_S);
    Serial.print(F("MOTOR_RPM="));       Serial.println(MOTOR_RPM, 3);
    Serial.print(F("CAL_FACTOR="));      Serial.println(CAL_FACTOR, 4);
    Serial.print(F("STEPS_PER_REV="));   Serial.println(STEPS_PER_REV);
    Serial.print(F("LEADSCREW_LEAD_MM=")); Serial.println(LEADSCREW_LEAD_MM, 3);
    Serial.print(F("SYRINGE_ID_MM="));   Serial.println(SYRINGE_ID_MM, 3);
    Serial.print(F("VOLUME_IS_TOTAL_MIXED=")); Serial.println(VOLUME_IS_TOTAL_MIXED ? "true" : "false");
    Serial.println(F("----------------"));
  } else {
    Serial.println(F("Unknown command. Try: STATUS, V <ml>, SEG <ml>, WAIT <s>, RPM <rpm>, CAL <x>, GO"));
  }

  Serial.println(F("OK"));
}

// Trigger detect (start when D8 goes HIGH)
bool triggerStartDetected() {
  static bool last = false;
  bool now = digitalRead(PIN_TRIGGER);

  if (now && !last) {
    delay(TRIGGER_DEBOUNCE_MS);
    if (digitalRead(PIN_TRIGGER)) {
      last = now;
      return true;
    }
  }
  last = now;
  return false;
}

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  pinMode(PIN_TRIGGER, INPUT); // if we need pull-down/up, add external resistor

  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR,  DIR_DISPENSE_LEVEL);
  setEnable(false);

  Serial.begin(115200);
  Serial.println(F("Resin Dispenser Ready."));
  Serial.println(F("Type STATUS for settings, GO to dispense, or trigger via D8 HIGH."));
}

void loop() {
  handleSerial();

  if (triggerStartDetected()) {
    Serial.println(F("TRIGGER received on D8 -> starting dispense..."));
    dispenseTotalMl(TARGET_TOTAL_ML);
  }
}
