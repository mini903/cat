import RPi.GPIO as GPIO
import time

# 🔧 너 회로도 기준 (MDDS60 핀 연결)
# GPIO23 → DIR (방향)
# GPIO24 → PWM (속도)

DIR_PIN = 23
PWM_PIN = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(PWM_PIN, GPIO.OUT)

pwm = GPIO.PWM(PWM_PIN, 1000)  # 1kHz PWM
pwm.start(0)

def drive_forward(speed=50):
    GPIO.output(DIR_PIN, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)

def drive_backward(speed=50):
    GPIO.output(DIR_PIN, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)

def stop():
    pwm.ChangeDutyCycle(0)

try:
    print("전진 2초")
    drive_forward(60)
    time.sleep(2)

    print("정지 1초")
    stop()
    time.sleep(1)

    print("후진 2초")
    drive_backward(60)
    time.sleep(2)

    print("정지")
    stop()

except KeyboardInterrupt:
    pass

finally:
    stop()
    pwm.stop()
    GPIO.cleanup()
