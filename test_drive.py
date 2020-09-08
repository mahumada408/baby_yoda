import evdev
import time
from threading import Thread
from queue import Queue
import collections
import numpy as np
from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO


dev = evdev.InputDevice('/dev/input/event2')
events = Queue()
event_deque = collections.deque()

ServoData = collections.namedtuple("ServoData", "current_angle previous_time rate_lim")
DrivePins = collections.namedtuple("DrivePins", "pwm1 pwm2 in1 in2 in3 in4")

def window_threshold(signal, lower, upper):
  if signal >= upper or signal <= lower:
    return True
  return False

def worker():
    for event in dev.read_loop():
      abs_check = window_threshold(event.value, 126, 128)
      if event.type == evdev.ecodes.EV_KEY or abs_check and event.type != evdev.ecodes.SYN_REPORT:
          # events.put(event)
          event_deque.append(event)

def main():

    kit = ServoKit(channels=16)
    counter = 0

    t = Thread(target=worker)
    t.start()

    ABS_X = 0
    ABS_Y = 0
    ABS_RX = 0
    ABS_RY = 0
    ABS_Z = 90
    ABS_RZ = 0

    num_servos = 9
    servo_angles = [90] * num_servos
    previous_servo_angles = [0] * num_servos

    yaw_data = ServoData(current_angle=0, previous_time=0, rate_lim=100)
    roll_data = ServoData(current_angle=0, previous_time=0, rate_lim=100)
    pitch_data = ServoData(current_angle=0, previous_time=0, rate_lim=100)
    left_shoulder_data = ServoData(current_angle=0, previous_time=0, rate_lim=200)
    right_shoulder_data = ServoData(current_angle=0, previous_time=0, rate_lim=200)

    start_time = time.clock()

    pwm1 = 16
    pwm2 = 13
    in1 = 20
    in2 = 21
    in3 = 5
    in4 = 6
    drive_pins = DrivePins(pwm1=16, pwm2=13, in1=20, in2=21, in3=5, in4=6)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pwm1, GPIO.OUT)
    GPIO.setup(pwm2, GPIO.OUT)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.setup(in3, GPIO.OUT)
    GPIO.setup(in4, GPIO.OUT)

    # Default to drive forward
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)

    pwm_freq = 1000
    max_duty = 50

    left = GPIO.PWM(pwm1, pwm_freq)
    right = GPIO.PWM(pwm2, pwm_freq)
    left.start(0) # Start at 0% duty cycle
    right.start(0) # Start at 0% duty cycle


    while True:
        # if not events.empty():
        if event_deque:
            # event = events.get_nowait()
            event = event_deque.popleft()
            if 'ABS_X' in str(evdev.categorize(event)):
                ABS_X = np.interp(float(event.value), [0,255], [-45,45])
            elif 'ABS_Y' in str(evdev.categorize(event)):
                ABS_Y = -np.interp(float(event.value), [0,255], [-max_duty,max_duty])
            elif 'ABS_RX' in str(evdev.categorize(event)):
                ABS_RX = np.interp(float(event.value), [0,255], [-max_duty,max_duty])
            elif 'ABS_RY' in str(evdev.categorize(event)):
                ABS_RY = -np.interp(float(event.value), [0,255], [-max_duty,max_duty])
            elif 'ABS_Z' in str(evdev.categorize(event)):
                # L2
                ABS_Z = np.interp(float(event.value), [0,255], [90,180])
            elif 'ABS_RZ' in str(evdev.categorize(event)):
                # R2
                ABS_RZ = np.interp(float(event.value), [0,255], [0,90])

        Drive(ABS_Y, ABS_RX, left, right, drive_pins)
        # left.ChangeDutyCycle(np.abs(ABS_RY))
        # right.ChangeDutyCycle(np.abs(ABS_RY))

def Drive(longitudinal_duty_cycle, lateral_duty_cycle, left_pwm, right_pwm, drive_pins):
    # Split up to left and right.
    if longitudinal_duty_cycle != 0:
        left_drive_command = np.sign(longitudinal_duty_cycle) * (longitudinal_duty_cycle + lateral_duty_cycle)
        right_drive_command = np.sign(longitudinal_duty_cycle) * (longitudinal_duty_cycle - lateral_duty_cycle)
    else:
        left_drive_command = lateral_duty_cycle
        right_drive_command = lateral_duty_cycle
    print("command: ", left_drive_command, end=' | ')
    print(right_drive_command)

    # Set driving direction (forward/back).    
    GPIO.output(drive_pins.in1, int(~(right_drive_command > 0)))
    GPIO.output(drive_pins.in2, int(right_drive_command > 0))
    GPIO.output(drive_pins.in4, int(~(left_drive_command > 0)))
    GPIO.output(drive_pins.in3, int(left_drive_command > 0))

    left_drive_command = np.clip(left_drive_command, -50, 50)
    right_drive_command = np.clip(right_drive_command, -50, 50)
    
    left_pwm.ChangeDutyCycle(np.abs(left_drive_command))
    right_pwm.ChangeDutyCycle(np.abs(right_drive_command))
    


if __name__ == '__main__':
    main()
