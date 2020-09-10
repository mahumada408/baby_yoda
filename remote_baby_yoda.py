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

    # Setup for drive system.
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
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

    pwm_freq = 1000
    max_duty = 100

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
                ABS_X = -np.interp(float(event.value), [0,255], [-max_duty,max_duty])
            elif 'ABS_Y' in str(evdev.categorize(event)):
                ABS_Y = -np.interp(float(event.value), [0,255], [-max_duty,max_duty])
            elif 'ABS_RX' in str(evdev.categorize(event)):
                ABS_RX = np.interp(float(event.value), [0,255], [-45,45])
            elif 'ABS_RY' in str(evdev.categorize(event)):
                ABS_RY = -np.interp(float(event.value), [0,255], [-60,60])
            elif 'ABS_Z' in str(evdev.categorize(event)):
                # L2
                ABS_Z = np.interp(float(event.value), [0,255], [90,180])
            elif 'ABS_RZ' in str(evdev.categorize(event)):
                # R2
                ABS_RZ = np.interp(float(event.value), [0,255], [0,90])

        yaw_data = CleanAngle(0, yaw_data.current_angle, yaw_data.previous_time, yaw_data.rate_lim)
        roll_data = CleanAngle(ABS_RX, roll_data.current_angle, roll_data.previous_time, roll_data.rate_lim)
        pitch_data = CleanAngle(ABS_RY, pitch_data.current_angle, pitch_data.previous_time, pitch_data.rate_lim)
        left_shoulder_data = CleanAngle(ABS_Z, left_shoulder_data.current_angle, left_shoulder_data.previous_time, left_shoulder_data.rate_lim)
        right_shoulder_data = CleanAngle(ABS_RZ, right_shoulder_data.current_angle, right_shoulder_data.previous_time, right_shoulder_data.rate_lim)

        servo_1_angle = np.clip(90 - pitch_data.current_angle + roll_data.current_angle, 0, 180)
        servo_2_angle = np.clip(90 + pitch_data.current_angle + roll_data.current_angle, 0, 180)
        servo_angles[0] = 90 + yaw_data.current_angle
        servo_angles[1] = servo_1_angle
        servo_angles[2] = servo_2_angle
        servo_angles[4] = left_shoulder_data.current_angle
        servo_angles[6] = 180 - left_shoulder_data.current_angle
        servo_angles[3] = right_shoulder_data.current_angle
        servo_angles[5] = 90 - right_shoulder_data.current_angle

        # Writes at 100Hz
        if (time.clock() - start_time) >= 0.01:
            ServoWrite(kit, servo_angles)
            Drive(ABS_Y, ABS_X, left, right, drive_pins)
            start_time = time.clock()


# Algorithm from
# https://en.wikipedia.org/wiki/Exponential_smoothing
def filter(current_command, previous_command, smoothing_factor):
    return (smoothing_factor * current_command) + (1 - smoothing_factor) * previous_command

def clamp_servo_rate(previous_command, current_command, time_dt, rate_limit):
    current_rate = (current_command - previous_command) / time_dt
    if (abs(current_rate) > rate_limit):
        return previous_command + np.sign(current_rate) * rate_limit * time_dt
    else:
        return current_command

def ServoWrite(servos, servo_angles):
    for i in range(len(servo_angles)):
        servos.servo[i].angle = servo_angles[i]

def CleanAngle(raw_data, previous_angle, previous_time, rate_lim):
    current_time = time.clock()
    servo_angle = clamp_servo_rate(previous_angle, raw_data, (current_time - previous_time), rate_lim)
    servo_stuff = ServoData(current_angle=servo_angle, previous_time=current_time, rate_lim=rate_lim)

    return servo_stuff

def Drive(longitudinal_duty_cycle, lateral_duty_cycle, left_pwm, right_pwm, drive_pins):
    left_drive_command = longitudinal_duty_cycle + lateral_duty_cycle
    right_drive_command = longitudinal_duty_cycle - lateral_duty_cycle

    if WindowThresh(longitudinal_duty_cycle, 10):
        GPIO.output(drive_pins.in1, int(longitudinal_duty_cycle < 0))
        GPIO.output(drive_pins.in2, int(longitudinal_duty_cycle > 0))
        GPIO.output(drive_pins.in3, int(longitudinal_duty_cycle > 0))
        GPIO.output(drive_pins.in4, int(longitudinal_duty_cycle < 0))
    else:
        GPIO.output(drive_pins.in1, int(right_drive_command > 0))
        GPIO.output(drive_pins.in2, int(right_drive_command < 0))
        GPIO.output(drive_pins.in3, int(left_drive_command < 0))
        GPIO.output(drive_pins.in4, int(left_drive_command > 0))

    left_drive_command = np.clip(left_drive_command, -100, 100)
    right_drive_command = np.clip(right_drive_command, -100, 100)
    
    left_pwm.ChangeDutyCycle(np.abs(left_drive_command))
    right_pwm.ChangeDutyCycle(np.abs(right_drive_command))

def WindowThresh(signal, threshold):
    if signal >= np.abs(threshold) or signal <= -np.abs(threshold):
        return True
    return False


if __name__ == '__main__':
    main()
