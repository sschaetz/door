// -------- Pins --------
const int STEP_PIN = 2;
const int DIR_PIN  = 3;
const int ENA_PIN  = 4;
const int ENDSTOP_PIN = 7;

// -------- Config --------
const bool ENDSTOP_ACTIVE_LOW = false;
const bool HOME_DIRECTION_FORWARD = false;

const int FULL_STEPS = 200;     // 1.8° motor
const int MICROSTEPS = 8;       // must match driver DIP
const int GEAR_RATIO = 5;       // 5:1 gearbox

const float HOME_RPM = 5.0;     // slow homing
const float RUN_RPM  = 30.0;    // normal speed

// Speed profile for a "change"
const float RUN_RPM_BASE = 30.0;     // your existing RUN_RPM
const float RUN_RPM_END  = 45.0;     // final speed near end (try 35..50 carefully)
const float END_RAMP_FRAC = 0.25;    // last 25% of travel ramps up (0.10..0.40)
const int   RAMP_UPDATE_EVERY_STEPS = 8; // update speed every N steps (reduces float cost)

const unsigned int PULSE_HIGH_US = 5;

long pos_steps = 0;

// Safety: when moving away from home, endstop must clear within this travel
const long CLEAR_WITHIN_DEG = 10;   // adjust (5..20 deg typical)



// --- Time scaling for testing ---
// 1.0 = real time
// 1.0/60.0 => 30 minutes behaves like 30 seconds
const float TIME_SCALE = 1.0/6.0;

// Base period for 30 minutes (real-world)
const unsigned long PERIOD_30_MIN_MS_REAL = 30UL * 60UL * 1000UL;

// Computed scaled periods
unsigned long PERIOD_30_MS;
unsigned long PERIOD_15_MS;

// Schedule parameters
const int MIN_CHANGES_PER_30 = 2;
const int MAX_CHANGES_PER_30 = 6;

const unsigned long DELAY_BETWEEN_CHANGES_MIN_MS = 4UL * 1000UL;
const unsigned long DELAY_BETWEEN_CHANGES_MAX_MS = 20UL * 1000UL;

// Midpoint probability: 0..100
int MIDPOINT_CHANGE_PROB_PERCENT = 20;

// Motion endpoints
long maxPos = 0;
bool atMax = false; // start at home after homing

// Global schedule anchor (set after homing)
unsigned long epoch_ms = 0;

// Batch runner state
struct BatchState {
  bool active = false;
  int remaining = 0;
  unsigned long next_action_ms = 0;
};
BatchState batch;

// Trigger definition
struct Trigger {
  unsigned long offset_ms;    // within 30-min cycle
  uint8_t prob_percent;       // 0..100
  int min_changes;
  int max_changes;
  const char* name;
};

Trigger TRIGGERS[2]; // will fill in setup based on PERIOD_15_MS
const int NUM_TRIGGERS = 2;

// Track per-cycle execution
unsigned long current_cycle = 0xFFFFFFFFUL;
bool handled_in_cycle[2] = {false, false};

// -------- Helpers --------

long stepsPerOutputRev() {
  return (long)FULL_STEPS * MICROSTEPS * GEAR_RATIO;
}

long stepsForDeg(long deg) {
  return stepsPerOutputRev() * deg / 360;
}

unsigned long stepDelayForRPM(float rpm) {
  float steps_per_sec = (rpm / 60.0) * stepsPerOutputRev();
  float period_us = 1000000.0 / steps_per_sec;
  long low_us = (long)(period_us - PULSE_HIGH_US);
  if (low_us < 5) low_us = 5;
  return low_us;
}

bool endstopActive() {
  int v = digitalRead(ENDSTOP_PIN);
  return ENDSTOP_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

void faultStop(const char* reason) {
  // Stop pulses and disable driver.
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(ENA_PIN, HIGH); // disable (common active-low)

  Serial.println();
  Serial.println("==== FAULT STOP ====");
  Serial.println(reason);
  Serial.println("====================");

  // Optional: blink LED on pin 13 so you see it's in fault
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

void stepOnce(unsigned long low_us, bool forward) {
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(PULSE_HIGH_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(low_us);

  pos_steps += forward ? 1 : -1;
}

// -------- Homing --------

void home() {
  unsigned long low_us = stepDelayForRPM(HOME_RPM);

  digitalWrite(DIR_PIN, HOME_DIRECTION_FORWARD ? HIGH : LOW);

  while (!endstopActive()) {
    stepOnce(low_us, HOME_DIRECTION_FORWARD);
  }

  pos_steps = 0;  // define home position
}

// -------- Move to position --------

void moveTo(long target, float rpm) {
  unsigned long low_us = stepDelayForRPM(rpm);
  bool forward = (target > pos_steps);

  // Safety check only when moving away from home (pos increasing)
  const bool movingAwayFromHome = forward && (pos_steps == 0);

  long clear_deadline = 0;
  if (movingAwayFromHome) {
    // If we're sitting on the endstop, it must clear soon after we start moving away
    if (endstopActive()) {
      clear_deadline = pos_steps + stepsForDeg(CLEAR_WITHIN_DEG);
    }
  }

  while (pos_steps != target) {
    stepOnce(low_us, forward);

    // Enforce: endstop must become inactive soon when moving away
    if (movingAwayFromHome && clear_deadline != 0) {
      if (!endstopActive()) {
        clear_deadline = 0; // cleared -> OK
      } else if (pos_steps >= clear_deadline) {
        faultStop("Endstop did not clear while moving away");
      }
    }
  }
}

void moveToWithEndRamp(long target, float rpm_base, float rpm_end, float end_ramp_frac) {
  bool forward = (target > pos_steps);
  long start_pos = pos_steps;
  long total = (target > start_pos) ? (target - start_pos) : (start_pos - target);
  if (total == 0) return;

  // Safety check only when moving away from home (pos increasing)
  const bool movingAwayFromHome = forward && (pos_steps == 0);

  long clear_deadline = 0;
  if (movingAwayFromHome) {
    if (endstopActive()) {
      clear_deadline = pos_steps + stepsForDeg(CLEAR_WITHIN_DEG);
    }
  }

  // Define where ramp begins (in steps from start)
  long ramp_len = (long)(total * end_ramp_frac);
  if (ramp_len < 1) ramp_len = 1;
  long ramp_start = total - ramp_len;  // steps_done >= ramp_start => ramp zone

  unsigned long low_us = stepDelayForRPM(rpm_base);

  long steps_done = 0;
  while (pos_steps != target) {
    // Update speed occasionally (cheaper than every step)
    if ((steps_done % RAMP_UPDATE_EVERY_STEPS) == 0) {
      if (steps_done >= ramp_start) {
        // progress through ramp zone: 0..1
        float t = (float)(steps_done - ramp_start) / (float)ramp_len;
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        // Linear ramp from base -> end
        float rpm = rpm_base + (rpm_end - rpm_base) * t;
        low_us = stepDelayForRPM(rpm);
      } else {
        low_us = stepDelayForRPM(rpm_base);
      }
    }

    stepOnce(low_us, forward);
    steps_done++;

    // Enforce: endstop must become inactive soon when moving away
    if (movingAwayFromHome && clear_deadline != 0) {
      if (!endstopActive()) {
        clear_deadline = 0; // cleared -> OK
      } else if (pos_steps >= clear_deadline) {
        faultStop("Endstop did not clear while moving away");
      }
    }
  }
}

// ============================================================
// SCHEDULER (robust, multi-timer, cycle-position based)
// ============================================================


unsigned long randBetweenUL(unsigned long lo, unsigned long hi_inclusive) {
  if (hi_inclusive <= lo) return lo;
  return lo + (unsigned long)random((long)(hi_inclusive - lo + 1));
}

int randBetweenInt(int lo, int hi_inclusive) {
  if (hi_inclusive <= lo) return lo;
  return lo + (int)random((long)(hi_inclusive - lo + 1));
}

void printNowTag() {
  Serial.print("[t=");
  Serial.print(millis());
  Serial.print("] ");
}

void updatePeriodConstants() {
  PERIOD_30_MS = (unsigned long)(PERIOD_30_MIN_MS_REAL * TIME_SCALE);
  if (PERIOD_30_MS < 1000UL) PERIOD_30_MS = 1000UL;

  PERIOD_15_MS = PERIOD_30_MS / 2;
  if (PERIOD_15_MS < 500UL) PERIOD_15_MS = 500UL;
}

void doOneChange() {
  long target = atMax ? 0 : maxPos;

  printNowTag();
  Serial.print("CHANGE: moving to ");
  Serial.println(target == 0 ? "HOME (0)" : "MAX");

  moveToWithEndRamp(target, RUN_RPM_BASE, RUN_RPM_END, END_RAMP_FRAC);
  atMax = !atMax;
}

void startBatch(const Trigger& trig) {
  int n = randBetweenInt(trig.min_changes, trig.max_changes);
  batch.active = true;
  batch.remaining = n;
  batch.next_action_ms = millis(); // start immediately

  printNowTag();
  Serial.print(trig.name);
  Serial.print(": START batch, planned changes=");
  Serial.println(n);
}

void runBatchStep() {
  unsigned long now = millis();
  if ((long)(now - batch.next_action_ms) < 0) return;

  if (batch.remaining <= 0) {
    batch.active = false;
    printNowTag();
    Serial.println("Batch done.");
    return;
  }

  doOneChange();
  batch.remaining--;

  if (batch.remaining > 0) {
    unsigned long d = randBetweenUL(DELAY_BETWEEN_CHANGES_MIN_MS, DELAY_BETWEEN_CHANGES_MAX_MS);
    batch.next_action_ms = millis() + d;

    printNowTag();
    Serial.print("Batch: remaining=");
    Serial.print(batch.remaining);
    Serial.print(" next change in ");
    Serial.print(d);
    Serial.println(" ms");
  } else {
    batch.active = false;
    printNowTag();
    Serial.println("Batch done.");
  }
}

void resetCycleState(unsigned long cycle_idx) {
  current_cycle = cycle_idx;
  for (int i = 0; i < NUM_TRIGGERS; i++) handled_in_cycle[i] = false;

  printNowTag();
  Serial.print("Entered cycle ");
  Serial.println(cycle_idx);
}

void considerTrigger(int idx, unsigned long pos_in_cycle) {
  const Trigger& trig = TRIGGERS[idx];

  if (handled_in_cycle[idx]) return;
  if (pos_in_cycle < trig.offset_ms) return; // not due yet

  // Mark as handled for this cycle (even if we skip), so we don't spam
  handled_in_cycle[idx] = true;

  if (batch.active) {
    printNowTag();
    Serial.print(trig.name);
    Serial.println(": SKIP (batch active)");
    return;
  }

  int r = random(100); // 0..99
  bool do_it = (r < trig.prob_percent);

  printNowTag();
  Serial.print(trig.name);
  Serial.print(": due, r=");
  Serial.print(r);
  Serial.print(" prob=");
  Serial.print(trig.prob_percent);
  Serial.print("% => ");
  Serial.println(do_it ? "START" : "skip");

  if (do_it) {
    startBatch(trig);
  }
}

// -------- Setup --------

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);

  digitalWrite(ENA_PIN, LOW);  // enable (common active-low)

  Serial.begin(115200);
  delay(200);

  // Seed randomness (best if A0 is floating/unconnected; otherwise pick another source)
  randomSeed(analogRead(A0));

  // Motor endpoint
  maxPos = stepsForDeg(230);

  // Home first
  home();
  atMax = false;

  // Scheduler init
  updatePeriodConstants();

  // Define triggers (offsets are within each PERIOD_30_MS cycle)
  TRIGGERS[0] = { 0UL,         100, (int)MIN_CHANGES_PER_30, (int)MAX_CHANGES_PER_30, "T30" };
  TRIGGERS[1] = { PERIOD_15_MS, (uint8_t)MIDPOINT_CHANGE_PROB_PERCENT, 1, 1, "T15" };

  // Anchor time AFTER homing so the schedule is stable
  epoch_ms = millis();
  resetCycleState(0);

  printNowTag();
  Serial.print("Scaled PERIOD_30_MS=");
  Serial.print(PERIOD_30_MS);
  Serial.print(" PERIOD_15_MS=");
  Serial.println(PERIOD_15_MS);
}

// -------- Main loop --------

void loop() {
  unsigned long now = millis();

  // 1) If a batch is active, run it step-by-step
  if (batch.active) {
    runBatchStep();
    return;
  }

  // 2) Robust global schedule (no drift, safe across millis wrap)
  unsigned long elapsed = now - epoch_ms;
  unsigned long cycle_idx = elapsed / PERIOD_30_MS;
  unsigned long pos_in_cycle = elapsed % PERIOD_30_MS;

  if (cycle_idx != current_cycle) {
    resetCycleState(cycle_idx);
  }

  // 3) Evaluate triggers (unified logic)
  for (int i = 0; i < NUM_TRIGGERS; i++) {
    considerTrigger(i, pos_in_cycle);
  }

  // idle
}


