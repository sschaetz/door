// -------- Pins --------
const int STEP_PIN = 2;
const int DIR_PIN  = 3;
const int ENA_PIN  = 4;
const int ENDSTOP_PIN = 7;

// -------- Config --------
const bool ENDSTOP_ACTIVE_LOW = false;
const bool HOME_DIRECTION_FORWARD = true;

const int FULL_STEPS = 200;     // 1.8° motor
const int MICROSTEPS = 8;       // must match driver DIP
const int GEAR_RATIO = 5;       // 5:1 gearbox

const float HOME_RPM = 5.0;     // slow homing
const float RUN_RPM  = 30.0;    // normal speed

const unsigned int PULSE_HIGH_US = 5;

long pos_steps = 0;

// -------- Helpers --------

long stepsPerOutputRev() {
  return (long)FULL_STEPS * MICROSTEPS * GEAR_RATIO;
}

long stepsFor180deg() {
  return stepsPerOutputRev() / 2;
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

  while (pos_steps != target) {
    stepOnce(low_us, forward);
  }
}

// -------- Setup --------

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);

  digitalWrite(ENA_PIN, LOW);  // enable (common active-low)

  delay(200);

  home();
}

// -------- Main loop --------

void loop() {
  long maxPos = stepsFor180deg();

  moveTo(maxPos, RUN_RPM);  // go 180° away
  delay(300);

  moveTo(0, RUN_RPM);       // return home
  delay(300);
}

