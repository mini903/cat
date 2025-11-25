#!/usr/bin/env python3
"""
autonomy_pi.py
- Pixhawk의 RC_CHANNELS를 받아 라즈베리파이로 Motor PWM과 Steering servo 제어
- 안전장치: kill switch, mav heartbeat 타임아웃
- 수정 포인트: SERIAL_PORT, baud, 핀 번호, RC 채널 매핑, PWM 범위
"""

import time
import threading
from pymavlink import mavutil
import RPi.GPIO as GPIO

# ---------------- Configuration ----------------
SERIAL_PORT = "/dev/serial0"    # Pixhawk ↔ RPi 연결 포트 (환경에 맞게 변경)
SERIAL_BAUD = 921600

# BCM pin numbers
MOTOR_PWM_PIN = 18     # PWM (hardware PWM 권장)
MOTOR_DIR_PIN = 17     # direction (optional)
STEER_PWM_PIN = 23     # servo pwm (50Hz)
KILL_SWITCH_PIN = 4    # kill switch input (pullup)

# RC channel mapping (1-based RC channels)
RC_STEER_IDX = 0   # RC channel 1 -> index 0
RC_THROTTLE_IDX = 2 # RC channel 3 -> index 2

# RC PWM expected range (typical)
RC_MIN = 1000
RC_MID = 1500
RC_MAX = 2000

# Motor PWM output range (duty cycle percent or PWM duty)
MOTOR_PWM_FREQ = 1000   # motor controller expects high-frequency PWM (check SmartDrive spec)
MOTOR_PWM_MIN = 0       # 0% duty
MOTOR_PWM_MAX = 100     # 100% duty

# Steering servo settings (50Hz servo)
STEER_PWM_FREQ = 50
STEER_ANGLE_LEFT = 1000   # microsecond pulse for left (adjust)
STEER_ANGLE_CENTER = 1500
STEER_ANGLE_RIGHT = 2000

HEARTBEAT_TIMEOUT = 1.0  # seconds without heartbeat -> stop
UPDATE_INTERVAL = 0.02   # control loop interval

# Safety
ENABLE_DIRECTION_CONTROL = True  # if SmartDrive uses direction pin(s)

# ------------------------------------------------

# Global state
last_heartbeat_time = 0.0
last_rc = None
running = True

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(KILL_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MOTOR_DIR_PIN, GPIO.OUT)
# PWM outputs
motor_pwm = GPIO.PWM(MOTOR_PWM_PIN, MOTOR_PWM_FREQ)
steer_pwm = GPIO.PWM(STEER_PWM_PIN, STEER_PWM_FREQ)
motor_pwm.start(0)  # 0% duty initially
steer_pwm.start(7.5)  # neutral approximate (will be updated)

# Helper functions
def rc_to_normalized(rc_value):
    """Convert RC PWM (1000..2000) to -1..1"""
    v = float(rc_value)
    # clamp
    v = max(RC_MIN, min(RC_MAX, v))
    # normalized -1 (min) to +1 (max)
    return (v - RC_MID) / (RC_MAX - RC_MIN) * 2.0

def set_motor_speed(normalized):
    """
    normalized: -1..1 (negative -> reverse if supported)
    Map to motor PWM duty (0..100) and direction pin
    """
    # clamp
    if normalized > 1.0: normalized = 1.0
    if normalized < -1.0: normalized = -1.0

    duty = abs(normalized) * MOTOR_PWM_MAX
    if duty < MOTOR_PWM_MIN:
        duty = 0

    # direction
    if ENABLE_DIRECTION_CONTROL:
        if normalized >= 0:
            GPIO.output(MOTOR_DIR_PIN, GPIO.HIGH)
        else:
            GPIO.output(MOTOR_DIR_PIN, GPIO.LOW)

    motor_pwm.ChangeDutyCycle(duty)

def set_steering(normalized):
    """
    normalized: -1(left) .. 0(center) .. +1(right)
    Map to servo pulse width microseconds, but RPi.GPIO PWM uses duty %
    We'll map microseconds to duty% at 50Hz:
    duty_percent = pulse_us / 20000 * 100  (period = 20ms)
    """
    pulse_mid = STEER_ANGLE_CENTER
    pulse = pulse_mid + normalized * (STEER_ANGLE_RIGHT - STEER_ANGLE_CENTER) if normalized >=0 else pulse_mid + normalized * (STEER_ANGLE_CENTER - STEER_ANGLE_LEFT)
    # compute duty %
    duty = (pulse / 20000.0) * 100.0
    steer_pwm.ChangeDutyCycle(duty)

def stop_all():
    motor_pwm.ChangeDutyCycle(0)
    # center steer
    center_duty = (STEER_ANGLE_CENTER / 20000.0) * 100.0
    steer_pwm.ChangeDutyCycle(center_duty)

# MAVLink handling thread
def mavlink_thread():
    global last_heartbeat_time, last_rc, running
    print("Connecting to MAVLink on", SERIAL_PORT, "baud", SERIAL_BAUD)
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=SERIAL_BAUD, source_system=255)
    # wait for heartbeat
    try:
        master.wait_heartbeat(timeout=5)
        print("MAVLink heartbeat detected from system %u comp %u" % (master.target_system, master.target_component))
    except Exception as e:
        print("No heartbeat detected:", e)
        # still continue; we'll monitor timeouts

    while running:
        try:
            msg = master.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            t = time.time()
            if msg.get_type() == 'HEARTBEAT':
                last_heartbeat_time = t
            elif msg.get_type() in ('RC_CHANNELS','RC_CHANNELS_RAW'):
                # RC_CHANNELS has fields chan1_raw..chan18_raw or .chanX
                # best to use msg.chan1_raw etc, but this depends on message
                # We'll try common attributes
                try:
                    # new style: channels is list
                    chans = []
                    if hasattr(msg, 'chan1_raw'):
                        # RC_CHANNELS_RAW: chan1_raw .. chan8_raw
                        for i in range(1, 19):
                            attr = 'chan{}_raw'.format(i)
                            if hasattr(msg, attr):
                                val = getattr(msg, attr)
                                chans.append(val)
                            else:
                                break
                    elif hasattr(msg, 'channels'):
                        chans = list(msg.channels)
                    elif hasattr(msg, 'chan1_raw'):
                        # fallback
                        chans = [msg.chan1_raw]
                    # store
                    last_rc = chans
                except Exception as e:
                    # ignore parsing errors
                    pass
            # other messages ignored for now
        except Exception as e:
            print("MAV recv error:", e)
            time.sleep(0.1)

# Control loop thread
def control_loop():
    global running, last_heartbeat_time, last_rc
    last_heartbeat_time = time.time()
    while running:
        # safety: kill switch (active low with pullup)
        if GPIO.input(KILL_SWITCH_PIN) == GPIO.LOW:
            print("KILL SWITCH ACTIVATED! Stopping motors.")
            stop_all()
            time.sleep(0.2)
            continue

        # mav heartbeat timeout
        if time.time() - last_heartbeat_time > HEARTBEAT_TIMEOUT:
            # no mavlink heartbeat -> stop
            stop_all()
            # print once
            # print("MAVLink heartbeat timeout, motors stopped.")
            time.sleep(UPDATE_INTERVAL)
            continue

        # if we have RC channels, use them
        if last_rc and len(last_rc) > max(RC_STEER_IDX, RC_THROTTLE_IDX):
            try:
                rc_throttle = last_rc[RC_THROTTLE_IDX]
                rc_steer = last_rc[RC_STEER_IDX]
                thr_norm = rc_to_normalized(rc_throttle)
                steer_norm = rc_to_normalized(rc_steer)
                # thr_norm: -1..1 where forward maybe positive; adjust sign if reversed
                set_motor_speed(thr_norm)
                set_steering(steer_norm)
            except Exception as e:
                print("Control update error:", e)
        else:
            # no RC yet -> keep stopped or center steer
            stop_all()

        time.sleep(UPDATE_INTERVAL)

if __name__ == "__main__":
    try:
        t_mav = threading.Thread(target=mavlink_thread, daemon=True)
        t_ctrl = threading.Thread(target=control_loop, daemon=True)
        t_mav.start()
        t_ctrl.start()
        print("Autonomy controller running. Ctrl-C to exit.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        running = False
        stop_all()
        motor_pwm.stop()
        steer_pwm.stop()
        GPIO.cleanup()
        print("Clean exit.")
