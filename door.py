from machine import Pin, PWM
import time

# -------------------------
# Pico pin mapping
# -------------------------
IN1 = Pin(6, Pin.OUT)   # L298N IN1
IN2 = Pin(7, Pin.OUT)   # L298N IN2
ENA = PWM(Pin(8))       # L298N ENA (remove jumper on ENA!)

SW  = Pin(2, Pin.IN, Pin.PULL_DOWN)  # your switch: CLOSED=1, OPEN=0

# -------------------------
# Tuning knobs
# -------------------------
DEBOUNCE_MS      = 30
STOP_SETTLE_MS   = 80

BACKOFF_DUTY     = 48000   # low speed to gently unlatch (0..65535)
RUN_DUTY         = 64000   # normal running speed
RAMP_STEP        = 2000    # how fast to ramp speed
RAMP_DELAY_MS    = 10

BACKOFF_MIN_MS   = 150     # move at least this long before accepting "open"
OPEN_TIMEOUT_MS  = 3000    # safety stop if never opens

ENA.freq(20000)            # ~20 kHz PWM (quiet)
ENA.duty_u16(0)

# -------------------------
# Helpers
# -------------------------
def read_stable(pin: Pin, stable_ms: int) -> int:
    v = pin.value()
    t0 = time.ticks_ms()
    while True:
        v2 = pin.value()
        if v2 != v:
            v = v2
            t0 = time.ticks_ms()
        if time.ticks_diff(time.ticks_ms(), t0) >= stable_ms:
            return v
        time.sleep_ms(1)

def is_closed() -> bool:
    return read_stable(SW, DEBOUNCE_MS) == 1

def is_open() -> bool:
    return read_stable(SW, DEBOUNCE_MS) == 0

def set_dir_fwd():
    IN1.value(1); IN2.value(0)

def set_dir_rev():
    IN1.value(0); IN2.value(1)

def motor_stop():
    ENA.duty_u16(0)
    IN1.value(0); IN2.value(0)

def ramp_to(target: int):
    # ramp current duty to target duty (reliable starts/stops)
    cur = ENA.duty_u16()
    if target > cur:
        d = cur
        while d < target:
            d = min(target, d + RAMP_STEP)
            ENA.duty_u16(d)
            time.sleep_ms(RAMP_DELAY_MS)
    else:
        d = cur
        while d > target:
            d = max(target, d - RAMP_STEP)
            ENA.duty_u16(d)
            time.sleep_ms(RAMP_DELAY_MS)

def run(direction: str, duty: int):
    if direction == "FWD":
        set_dir_fwd()
    else:
        set_dir_rev()
    ramp_to(duty)

# -------------------------
# State machine
# -------------------------
direction = "FWD"   # current run direction
state = "RUN"

# Start running forward
run(direction, RUN_DUTY)
print("Start: RUN", direction)

while True:
    if state == "RUN":
        if is_closed():
            # Stop immediately
            motor_stop()
            print("Switch CLOSED -> STOP")
            time.sleep_ms(STOP_SETTLE_MS)

            # Flip direction and start gentle backoff
            direction = "REV" if direction == "FWD" else "FWD"
            run(direction, BACKOFF_DUTY)
            print("Backoff in", direction, "until switch opens")

            backoff_start = time.ticks_ms()
            state = "BACKOFF_WAIT_OPEN"

    elif state == "BACKOFF_WAIT_OPEN":
        now = time.ticks_ms()

        moved_long_enough = time.ticks_diff(now, backoff_start) >= BACKOFF_MIN_MS
        if moved_long_enough and is_open():
            # Switch open confirmed -> go to normal run speed
            print("Switch OPEN confirmed -> RUN", direction)
            ramp_to(RUN_DUTY)
            state = "RUN"

        if time.ticks_diff(now, backoff_start) >= OPEN_TIMEOUT_MS:
            motor_stop()
            print("ERROR: switch did not open (timeout). Motor STOP.")
            state = "FAULT"

    elif state == "FAULT":
        time.sleep_ms(200)

    time.sleep_ms(2)

