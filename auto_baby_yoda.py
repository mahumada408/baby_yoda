import collections
import evdev
import numpy as np
import time
import threading
from queue import Queue

from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO
import multiprocessing
import serial

import scripts.yoda_helper as yoda_helper
import hw.pin_assignments as yoda_pins


# dev = evdev.InputDevice('/dev/input/event2')
events = Queue()
event_deque = collections.deque()

ServoData = collections.namedtuple("ServoData", "current_angle previous_time rate_lim")
DrivePins = collections.namedtuple("DrivePins", "pwm1 pwm2 in1 in2 in3 in4")
DriveCommands = collections.namedtuple("DriveCommands", "current_command previous_command dt accel_limit")
AllCommands = collections.namedtuple("AllCommands", "left right")

MAX_DUTY = 100

# Serial com
ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)

def window_threshold(signal, lower, upper):
  if signal >= upper or signal <= lower:
    return True
  return False

def worker():
    while True:
        input_string = ""
        input_commands = []
        input_string = ser.readline().decode('utf-8')
        if "c" in input_string:
            # Control signal. Remove leading 'c' character.
            # input_string.replace('c', '')
            input_string = input_string[1:]
            input_string = input_string.replace("\n","")
            input_string = input_string.split(',')
            input_commands = [-int(command) for command in input_string]
            for i in range(2):
                input_commands.append(-int(input_string[i]))
            # print(input_commands)
            event_deque.append(input_commands)

def main():
    kit = ServoKit(channels=16)

    t = threading.Thread(target=worker)
    t.start()

    timestamp = time.clock()

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

    pwm_freq = 100
    max_duty = 50

    left = GPIO.PWM(pwm1, pwm_freq)
    right = GPIO.PWM(pwm2, pwm_freq)
    left.start(0) # Start at 0% duty cycle
    right.start(0) # Start at 0% duty cycle
    max_accel = 100
    left_drive_commands = DriveCommands(current_command=0, previous_command=0, dt=0.01, accel_limit=max_accel)
    right_drive_commands = DriveCommands(current_command=0, previous_command=0, dt=0.01, accel_limit=max_accel)
    all_commands = AllCommands(left=left_drive_commands, right=right_drive_commands)

    left_command = 0
    right_command = 0
    limit_rate = 0.75
    while True:
        if event_deque:
            commands = event_deque.popleft()
            left_command = commands[0]
            right_command = commands[1]
        # print(f"left: {left_command} right: {right_command}")
        # Writes at 100Hz
        if (time.clock() - timestamp) >= 0.01:
            # print(f"left: {left_command} right: {right_command}")
            all_commands = DriveAuto(right_command * limit_rate, left_command * limit_rate, left, right, drive_pins, all_commands)


# Algorithm from
# https://en.wikipedia.org/wiki/Exponential_smoothing
def filter(current_command, previous_command, smoothing_factor):
    return (smoothing_factor * current_command) + (1 - smoothing_factor) * previous_command

def ClampRate(previous_command, current_command, time_dt, rate_limit):
    current_rate = (current_command - previous_command) / time_dt
    # print(f"current accel: {current_rate}")
    if (abs(current_rate) > rate_limit):
        # print("Limiting accel!")
        return previous_command + np.sign(current_rate) * rate_limit * time_dt
    else:
        return current_command

def ServoWrite(servos, servo_angles):
    for i in range(len(servo_angles)):
        servos.servo[i].angle = servo_angles[i]
    return servo_angles.copy()

def CleanAngle(raw_data, previous_angle, previous_time, rate_lim):
    current_time = time.clock()
    servo_angle = ClampRate(previous_angle, raw_data, (current_time - previous_time), rate_lim)
    servo_stuff = ServoData(current_angle=servo_angle, previous_time=current_time, rate_lim=rate_lim)

    return servo_stuff

def DriveAuto(left_drive_command, right_drive_command, left_pwm, right_pwm, drive_pins, all_commands):
    GPIO.output(drive_pins.in1, int(right_drive_command > 0))
    GPIO.output(drive_pins.in2, int(right_drive_command <= 0))
    GPIO.output(drive_pins.in3, int(left_drive_command <= 0))
    GPIO.output(drive_pins.in4, int(left_drive_command > 0))

    # Interpolate between the Android's app's 192 max command to max pwm command. 
    max_android_app_command = 192
    max_pwm_command = 100
    left_drive_command = np.interp(left_drive_command, [-max_android_app_command, max_android_app_command], [-max_pwm_command, max_pwm_command])
    right_drive_command = np.interp(right_drive_command, [-max_android_app_command, max_android_app_command], [-max_pwm_command, max_pwm_command])

    left_drive_command_test = ClampRate(all_commands.left.previous_command, left_drive_command, all_commands.left.dt, all_commands.left.accel_limit)
    right_drive_command_test = ClampRate(all_commands.right.previous_command, right_drive_command, all_commands.right.dt, all_commands.right.accel_limit)

    left_drive_commands = DriveCommands(current_command=0, previous_command=left_drive_command_test, dt=all_commands.left.dt, accel_limit=all_commands.left.accel_limit)
    right_drive_commands = DriveCommands(current_command=0, previous_command=right_drive_command_test, dt=all_commands.right.dt, accel_limit=all_commands.right.accel_limit)
    
    new_all_commands = AllCommands(left=left_drive_commands, right=right_drive_commands)

    # print(f"original command: {left_drive_command}")
    print(f"clamped command: {np.abs(left_drive_command_test)} | {np.abs(right_drive_command_test)}")
    left_pwm.ChangeDutyCycle(np.clip(np.abs(left_drive_command_test), 0, MAX_DUTY))
    right_pwm.ChangeDutyCycle(np.clip(np.abs(right_drive_command_test), 0, MAX_DUTY))
    return new_all_commands

def WindowThresh(signal, threshold):
    if signal >= np.abs(threshold) or signal <= -np.abs(threshold):
        return True
    return False
def PlaybackRecording(servo_kit, servo_recording):
    servo_states = servo_recording[0]
    recording_start_time = servo_states[1]
    current_start_time = time.clock()
    servo_recording.pop(0)
    for servo_states in servo_recording:
        time1 = time.clock()
        for i in range(len(servo_states[0])):
            servo_kit.servo[i].angle = servo_states[0][i]
        print(f"serve 1: {servo_states[0][1]} | servo 2: {servo_states[0][2]}")
        time2 = time.clock()
        time.sleep((servo_states[1] - recording_start_time))
        recording_start_time = servo_states[1]
    print("done", flush=True)


if __name__ == '__main__':
    main()
