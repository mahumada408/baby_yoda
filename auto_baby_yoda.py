import evdev
import time
import threading
from queue import Queue
import collections
import numpy as np
from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO
import multiprocessing
import scripts.yoda_helper as yoda_helper


# dev = evdev.InputDevice('/dev/input/event2')
events = Queue()
event_deque = collections.deque()

ServoData = collections.namedtuple("ServoData", "current_angle previous_time rate_lim")
DrivePins = collections.namedtuple("DrivePins", "pwm1 pwm2 in1 in2 in3 in4")
DriveCommands = collections.namedtuple("DriveCommands", "current_command previous_command dt accel_limit")
AllCommands = collections.namedtuple("AllCommands", "left right")

# Serial com
ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)

def window_threshold(signal, lower, upper):
  if signal >= upper or signal <= lower:
    return True
  return False

def worker():
    while True:
        input_string = ""
        input_string = ser.readline().decode('utf-8')
        if "c" in state:
            # Control signal. Remove leading 'c' character.
            input_string.replace('c', '')
            input_string = input_string.split(',')
            event_deque.append(input_string)

def main():
    kit = ServoKit(channels=16)

    t = threading.Thread(target=worker)
    t.start()

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
    max_duty = 20

    left = GPIO.PWM(pwm1, pwm_freq)
    right = GPIO.PWM(pwm2, pwm_freq)
    left.start(0) # Start at 0% duty cycle
    right.start(0) # Start at 0% duty cycle
    max_accel = 50
    left_drive_commands = DriveCommands(current_command=0, previous_command=0, dt=0.01, accel_limit=max_accel)
    right_drive_commands = DriveCommands(current_command=0, previous_command=0, dt=0.01, accel_limit=max_accel)
    all_commands = AllCommands(left=left_drive_commands, right=right_drive_commands)

     while True:
        if event_deque:
            commands = event_deque.popleft()
            left_command = commands[0]
            right_command = commands[0]
        print(f"left: {left_command} right: {right_command}")
        # Writes at 100Hz
        if (time.clock() - timestamp) >= 0.01:
            all_commands = DriveAuto(left_command, right_command, left, right, drive_pins, all_commands)


# Algorithm from
# https://en.wikipedia.org/wiki/Exponential_smoothing
def filter(current_command, previous_command, smoothing_factor):
    return (smoothing_factor * current_command) + (1 - smoothing_factor) * previous_command

def ClampRate(previous_command, current_command, time_dt, rate_limit):
    current_rate = (current_command - previous_command) / time_dt
    if (abs(current_rate) > rate_limit):
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

def Drive(longitudinal_duty_cycle, lateral_duty_cycle, left_pwm, right_pwm, drive_pins, all_commands):
    left_drive_command = longitudinal_duty_cycle + lateral_duty_cycle
    right_drive_command = longitudinal_duty_cycle - lateral_duty_cycle

    if WindowThresh(longitudinal_duty_cycle, 15):
        # Mode for only driving straight with no steer
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

    left_drive_command_test = ClampRate(all_commands.left.previous_command, left_drive_command, all_commands.left.dt, all_commands.left.accel_limit)
    right_drive_command_test = ClampRate(all_commands.right.previous_command, right_drive_command, all_commands.right.dt, all_commands.right.accel_limit)

    left_drive_commands = DriveCommands(current_command=0, previous_command=left_drive_command_test, dt=all_commands.left.dt, accel_limit=all_commands.left.accel_limit)
    right_drive_commands = DriveCommands(current_command=0, previous_command=right_drive_command_test, dt=all_commands.right.dt, accel_limit=all_commands.right.accel_limit)
    
    new_all_commands = AllCommands(left=left_drive_commands, right=right_drive_commands)
    
    left_pwm.ChangeDutyCycle(np.abs(left_drive_command))
    right_pwm.ChangeDutyCycle(np.abs(right_drive_command))
    return new_all_commands

def DriveAuto(left_drive_command, right_drive_command, left_pwm, right_pwm, drive_pins, all_commands):
    GPIO.output(drive_pins.in1, int(right_drive_command > 0))
    GPIO.output(drive_pins.in2, int(right_drive_command < 0))
    GPIO.output(drive_pins.in3, int(left_drive_command < 0))
    GPIO.output(drive_pins.in4, int(left_drive_command > 0))

    left_drive_command = np.interp(left_drive_command, [-192,192], [-100,100])
    right_drive_command = np.interp(right_drive_command, [-192,192], [-100,100])

    left_drive_command_test = ClampRate(all_commands.left.previous_command, left_drive_command, all_commands.left.dt, all_commands.left.accel_limit)
    right_drive_command_test = ClampRate(all_commands.right.previous_command, right_drive_command, all_commands.right.dt, all_commands.right.accel_limit)

    left_drive_commands = DriveCommands(current_command=0, previous_command=left_drive_command_test, dt=all_commands.left.dt, accel_limit=all_commands.left.accel_limit)
    right_drive_commands = DriveCommands(current_command=0, previous_command=right_drive_command_test, dt=all_commands.right.dt, accel_limit=all_commands.right.accel_limit)
    
    new_all_commands = AllCommands(left=left_drive_commands, right=right_drive_commands)
    
    left_pwm.ChangeDutyCycle(np.abs(left_drive_command))
    right_pwm.ChangeDutyCycle(np.abs(right_drive_command))
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
