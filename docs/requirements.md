# Hybrid 3D Printer — Casting + Printer Integration Requirements (SW/HW)
**Project:** Hybrid 3D Printer (Team 1)  
**Scope:** Requirements for implementing resin casting features and integrating them with printer steps (G-code initiated workflow).  
**Out of scope:** Detailed mechanical/frame redesign (covered elsewhere), full CAD, BOM expansion (refer to project docs).

---

## 1. System Context (high level)
The system combines:
1) **FDM mold printing** (single-use, dissolvable/removable mold)  
2) **Resin mixing + dispensing** (two-part resin, syringe-based, static mixer)  
3) **Workflow integration** where the printer **initiates and controls the pouring sequence via G-code** after print completion.

The system must support:
- Minimum mold build volume **150 mm × 150 mm × 150 mm**
- Minimum casting wall thickness **2.0 mm** 
- Mold features such as sprues/gates/vents 
- Removable/dissolvable mold materials  
- Resin dispense accuracy **±5%**  
- Resin viscosity compatibility **250–1000 cPs** 
- Support **1:1** mixing ratio  
- System modes: **manual** and **semi-automated (G-code initiated)** 
- Safety: **emergency stop** and **ventilation** 
- Operator output of state: **printing / pouring / curing** 

---

## 2. Definitions
- **Printer Controller / Motherboard:** The SV06 (or chosen printer) mainboard that executes G-code.
- **Dispenser Controller:** Arduino Uno controlling the stepper driver for syringe actuation.
- **Trigger Signal:** A printer-controlled digital output that starts dispensing.
- **DM860i/DM806i:** Stepper driver (STEP/DIR/ENA) used for the dispensing motor.
- **Static mixer:** Disposable mixing nozzle that prevents curing inside permanent components.

---

## 3. Hardware Interfaces (Canonical Pinout + Electrical)
### 3.1 Arduino ↔ Stepper Driver (Dispense motor)
**Pin mapping (canonical / required):**
- **D2 → PUL+ (STEP)**  ✅ (Change request: *use D2, not D6*)  
- **D4 → DIR+**  
- **D7 → ENA+**  
- **GND → PUL−, DIR−, ENA− (common reference)**  

> NOTE: Current firmware must match this pinout. If code still defines STEP on D6, it must be updated to D2 before integration testing.

This matches the newer wiring schematic showing **D2/D4/D7** controlling driver inputs.

### 3.2 Printer ↔ Arduino (Trigger input)
- Printer shall provide a **single trigger output** (from an available controlled output such as a fan/MOSFET output or other motherboard-controlled line).
- Trigger shall be **level shifted** to an Arduino-safe input level (3.3V or 5V logic). The current schematic shows a **12V → 3.3V** conversion feeding the Arduino trigger input.
- Arduino trigger input (recommended canonical): **D8 = DISPENSE_NOW** (HIGH starts dispense).

### 3.3 Limit switches (end stops)
To prevent over-travel:
- System shall include **two limit switches** at fully extended and fully retracted positions.
- Wiring per current schematic indicates Arduino inputs **D3 and D5** used for end stops (NO contacts shown).
- Firmware shall implement **hard stop behavior** when a limit switch activates (see UC-05).

### 3.4 Power distribution (printer PSU powering dispenser)
The integrated design assumes powering dispenser electronics from the printer PSU:
- System total current draw (printer + dispenser) is analyzed at **~16.66 A**, with recommended 125% margin **~20.83 A**. 
- Wiring guidance selects **AWG10** for all components drawing power from the printer PSU due to the ~20.8 A max output requirement. 
- The design includes DC-DC conversion (example shown): **24V → 48V** for the stepper driver/motor and **24V → 5V** for Arduino logic. 

### 3.5 Safety enclosure and protective components
- Electrical system shall be mounted within an **electrical enclosure** (referenced under UL 508A context). 
- System shall include a **rectifier diode** to prevent back-feeding/power surge into PSU (sized to 150% expected load at its node).
- System shall include an **emergency stop** accessible to the operator. 

---

## 4. Software Interfaces
### 4.1 Firmware (Arduino dispenser controller)
Firmware shall provide:
- **Serial control** for manual mode (set volume, speed, calibration, run).
- **Digital trigger input** to start dispense when printer requests (semi-auto mode).
- Conversion from **mL → plunger travel → motor steps** using:
  - Syringe inner diameter (nominal example: 50.8 mm referenced in project analysis)
  - Leadscrew lead (example: T8x2 = 2.0 mm/rev)
  - MICROSTEPS matching driver DIP settings
  - Calibration factor (tunable)

### 4.2 Printer-side G-code integration
Printer workflow shall:
- Finish mold print
- Park/move to fill position
- Issue a **G-code controlled output change** that raises the trigger line
- Wait for dispensing completion (v1: fixed dwell; v2: handshake)
- Continue to next step (curing dwell, cooldown, notify operator)

Requirement basis: semi-automated mode where **G-code initiates and controls pouring sequence** after print completion.

---

## 5. Use Cases & Requirements

### UC-01 — Manual dispense (calibration & bench testing)
**Goal:** Operator dispenses a specified total mixed volume without printer involvement.  
**Preconditions:** Dispenser powered; resin/water loaded; mixer/nozzle installed; safe area.

**Functional requirements**
- **UC-01-FR1:** Firmware shall accept a target volume in mL and execute dispensing to that target.
- **UC-01-FR2:** Firmware shall provide a calibration parameter to adjust mL-to-steps mapping.
- **UC-01-FR3:** Firmware shall support printing/logging computed travel and steps (for validation).

**Acceptance criteria**
- Dispensed volume error ≤ **±5%**.

---

### UC-02 — Printer-initiated dispense (semi-automated)
**Goal:** After mold print completes, the printer triggers dispensing via G-code.  
**Preconditions:** Mold printed; nozzle positioned at fill location; trigger wiring verified.

**Functional requirements**
- **UC-02-FR1:** Printer shall provide a controllable trigger output used to start dispensing.
- **UC-02-FR2:** Trigger signal shall be level-shifted to Arduino-safe logic input. 
- **UC-02-FR3:** Arduino shall detect a rising-edge trigger (debounced) and begin dispense sequence.
- **UC-02-FR4:** System shall operate in semi-automated mode where **G-code initiates/controls pouring**.

**Acceptance criteria**
- Trigger causes dispensing within 1 second of signal assertion (debounce excluded).
- No false triggers during printing.

---

### UC-03 — Interval pours (thermal management)
**Goal:** Dispense resin in segments (e.g., 50–100 mL) with waits between pours to reduce exotherm risk.  
**Preconditions:** Resin selected; segment size configured; curing constraints known.

**Functional requirements**
- **UC-03-FR1:** Firmware shall support a segment size (mL) and wait interval (seconds) between segments.
- **UC-03-FR2:** Firmware shall stop dispensing after reaching total target volume.
- **UC-03-FR3:** Segment/wait parameters shall be configurable via serial (manual mode) and/or preset profile.

**Acceptance criteria**
- Total dispensed volume meets **±5%** accuracy requirement.

---

### UC-04 — Abort / Emergency stop
**Goal:** Operator can immediately stop motion and make the system safe.  
**Preconditions:** System is dispensing or armed.

**Functional requirements**
- **UC-04-FR1:** System shall include an emergency stop. :contentReference
- **UC-04-FR2:** E-stop actuation shall remove power from the stepper driver enable/power path so motion ceases.
- **UC-04-FR3:** Firmware shall transition to a safe state after an abort (disable driver, ignore triggers until reset).

**Acceptance criteria**
- Pressing E-stop stops motion in < 0.5 s and prevents restart until explicit reset.

---

### UC-05 — Homing / travel limit protection (end stops)
**Goal:** Prevent over-travel and mechanical damage.  
**Preconditions:** Limit switches installed and wired.

**Functional requirements**
- **UC-05-FR1:** System shall include two end stops (fully extended & fully retracted). 
- **UC-05-FR2:** Firmware shall stop motion immediately if a limit switch is activated during movement.
- **UC-05-FR3:** Firmware shall optionally support a “home retract” routine to reach known start position.

**Acceptance criteria**
- Forcing a limit switch during motion stops stepping within ≤ 10 steps.

---

### UC-06 — Purge / cleanup between cycles
**Goal:** Make system usable for repeated cycles (minimize resin contact in permanent components).  
**Preconditions:** Disposable nozzle available; purge container available.

**Functional requirements**
- **UC-06-FR1:** System shall be designed for easy purge/clean between cycles. 
- **UC-06-FR2:** Firmware shall provide a purge mode (e.g., dispense X mL to waste, then retract).
- **UC-06-FR3:** System shall support quick replacement of the static mixing nozzle (disposable).

**Acceptance criteria**
- Purge mode executes without disassembly of syringes/driver wiring.

---

## 6. Performance Requirements
- **PR-01:** Dispense accuracy: **±5% volumetric**.
- **PR-02:** Resin viscosity compatibility: **250–1000 cPs**.  
- **PR-03:** Mixing ratio support: **1:1**.
- **PR-04:** Full sequence (mold fabrication + casting + cure to demoldable): **≤ 24 hours**.
- **PR-05:** Operator state reporting: printing/pouring/curing. 

---

## 7. Electrical/Compliance Requirements (implementation constraints)
- **EC-01:** Overcurrent protection design margin shall follow the 125% factor used in analysis. :contentReference
- **EC-02:** High-current supply wiring shall follow AWG10 guidance for the ~20.8A envelope. :contentReference
- **EC-03:** Enclosure shall be designed with consideration for ventilation/cooling recommendations due to unknown component temperatures.

---

## 8. Verification Plan (minimum)
- **VT-01 (Volumetric):** Water test at 50/100/200 mL; compute percent difference; must be within ±5%.
- **VT-02 (Trigger):** Execute end-of-print G-code trigger; confirm Arduino detects rising edge and runs dispense.
- **VT-03 (Limits):** Manually activate each limit switch during motion; confirm immediate stop and safe lockout.
- **VT-04 (E-stop):** Trigger E-stop mid-dispense; confirm power removed from motor drive path and motion ceases.
- **VT-05 (Power):** Measure current draw during worst case and validate within supply/fusing assumptions (per analysis envelope).

---

## 9. Implementation Notes (pin-change + alignment)
- **STEP pin is D2 (required).** If any prior documentation or code used D6, update firmware constants and verify continuity.
- Wiring schematic reference shows D2/D4/D7 for STEP/DIR/ENA and D8 trigger, with D3/D5 reserved for limit switches. 