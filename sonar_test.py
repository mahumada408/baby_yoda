import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

TRIG = 23
ECHO = 24

SPEED_OF_SOUND = 342000 # [mm/s]

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, False)
print("Waiting for sensor to settle")
time.sleep(2)

while True:
  # The HC-SR04 sensor requires a short 10uS pulse to trigger the module.
  GPIO.output(TRIG, True)
  time.sleep(0.00001)
  GPIO.output(TRIG, False)

  while GPIO.input(ECHO) == 0:
    pulse_start = time.time()

  while GPIO.input(ECHO) == 1:
    pulse_end = time.time()

  pulse_duration = pulse_end - pulse_start
  distance_mm = pulse_duration * SPEED_OF_SOUND/2
  print(f"distance: {distance_mm}")