import evdev
import time
import threading
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
DriveCommands = collections.namedtuple("DriveCommands", "current_command previous_command dt accel_limit")
AllCommands = collections.namedtuple("AllCommands", "left right")

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

    t = threading.Thread(target=worker)
    t.start()

    ABS_X = 0
    ABS_Y = 0
    ABS_RX = 0
    ABS_RY = 0
    ABS_Z = 0
    ABS_RZ = 0
    r1_held = False
    l1_held = False
    square_held = False
    o_held = False
    x_held = 1
    yaw = 0
    elbow = 90
    shoulder_1 = 45
    shoulder_2 = 90
    up_down = 0
    left_right = 0

    num_servos = 9
    servo_angles = [90] * num_servos

    yaw_data = ServoData(current_angle=0, previous_time=0, rate_lim=100)
    roll_data = ServoData(current_angle=0, previous_time=0, rate_lim=100)
    pitch_data = ServoData(current_angle=0, previous_time=0, rate_lim=100)
    shoulder_1_data = ServoData(current_angle=shoulder_1, previous_time=0, rate_lim=200)
    shoulder_2_data = ServoData(current_angle=shoulder_2, previous_time=0, rate_lim=200)
    elbow_data = ServoData(current_angle=0, previous_time=0, rate_lim=200)

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
    max_duty = 20

    left = GPIO.PWM(pwm1, pwm_freq)
    right = GPIO.PWM(pwm2, pwm_freq)
    left.start(0) # Start at 0% duty cycle
    right.start(0) # Start at 0% duty cycle
    max_accel = 50
    left_drive_commands = DriveCommands(current_command=0, previous_command=0, dt=0.01, accel_limit=max_accel)
    right_drive_commands = DriveCommands(current_command=0, previous_command=0, dt=0.01, accel_limit=max_accel)
    all_commands = AllCommands(left=left_drive_commands, right=right_drive_commands)

    # servo_recording = collections.deque()
    servo_recording = []
    record_servos = False
    last_r1 = 0
    triangle = False

    while True:
        # if not events.empty():
        if event_deque:
            # event = events.get_nowait()
            event = event_deque.popleft()
            if 'ABS_X' in str(evdev.categorize(event)):
                max_duty = 50
                ABS_X = -np.interp(float(event.value), [0,255], [-max_duty,max_duty])
            elif 'ABS_Y' in str(evdev.categorize(event)):
                max_duty = 20
                ABS_Y = -np.interp(float(event.value), [0,255], [-max_duty,max_duty])
            elif 'ABS_RX' in str(evdev.categorize(event)):
                ABS_RX = np.interp(float(event.value), [0,255], [-45,45])
            elif 'ABS_RY' in str(evdev.categorize(event)):
                ABS_RY = -np.interp(float(event.value), [0,255], [-60,60])
            elif 'ABS_Z' in str(evdev.categorize(event)):
                # L2
                # ABS_Z = np.interp(float(event.value), [0,255], [90,180])
                max_duty = 50
                ABS_Z = np.interp(float(event.value), [0,255], [0,x_held * max_duty])
            elif 'ABS_RZ' in str(evdev.categorize(event)):
                # R2
                ABS_RZ = np.interp(float(event.value), [0,255], [10,90])
            elif 'BTN_TR' in str(evdev.categorize(event)):
                # R1
                if int(event.value) and int(event.code) == 311:
                    record_servos = ~record_servos
            elif 'BTN_TL' in str(evdev.categorize(event)):
                # L1
                if int(event.value) and int(event.code) == 310:
                    l1_held = True
                else:
                    l1_held = False
            elif 'BTN_A' in str(evdev.categorize(event)):
                # x
                if int(event.value):
                    x_held = -1
                else:
                    x_held = 1
            elif 'BTN_Y' in str(evdev.categorize(event)):
                # square
                square_held = True if int(event.value) else False
            elif 'BTN_X' in str(evdev.categorize(event)):
                # triangle
                triangle = ~triangle if int(event.value) else triangle
            elif 'BTN_B' in str(evdev.categorize(event)):
                # circle
                o_held = True if int(event.value) else False
            elif 'ABS_HAT0Y' in str(evdev.categorize(event)):
                # d pad up down
                up_down = float(event.value)
            elif 'ABS_HAT0X' in str(evdev.categorize(event)):
                # d pad right left
                left_right = float(event.value)

        if r1_held:
            yaw = np.clip(yaw + 0.1, -45, 45)
        if l1_held:
            yaw = np.clip(yaw - 0.1, -45, 45)
        if square_held:
            elbow = np.clip(elbow + 0.05, 0, 180)
        if o_held:
            elbow = np.clip(elbow - 0.05, 0, 180)
        if up_down != 0:
            shoulder_2 = np.clip(shoulder_2 - up_down/20, 45, 135)
        if left_right != 0:
            shoulder_1 = np.clip(shoulder_1 - left_right/20, 10, 90)

        yaw_data = CleanAngle(yaw, yaw_data.current_angle, yaw_data.previous_time, yaw_data.rate_lim)
        roll_data = CleanAngle(ABS_RX, roll_data.current_angle, roll_data.previous_time, roll_data.rate_lim)
        pitch_data = CleanAngle(ABS_RY, pitch_data.current_angle, pitch_data.previous_time, pitch_data.rate_lim)
        shoulder_1_data = CleanAngle(shoulder_1, shoulder_1_data.current_angle, shoulder_1_data.previous_time, shoulder_1_data.rate_lim)
        shoulder_2_data = CleanAngle(shoulder_2, shoulder_2_data.current_angle, shoulder_2_data.previous_time, shoulder_2_data.rate_lim)
        elbow_data = CleanAngle(elbow, elbow_data.current_angle, elbow_data.previous_time, elbow_data.rate_lim)

        servo_1_angle = np.clip(90 - pitch_data.current_angle + roll_data.current_angle, 0, 180)
        servo_2_angle = np.clip(90 + pitch_data.current_angle + roll_data.current_angle, 0, 180)
        servo_angles[0] = 90 + yaw_data.current_angle
        servo_angles[1] = servo_1_angle
        servo_angles[2] = servo_2_angle

        # Shoulder 1
        servo_angles[3] = shoulder_1_data.current_angle
        servo_angles[6] = 90 - shoulder_1_data.current_angle

        # Shoulder 2
        servo_angles[4] = shoulder_2_data.current_angle
        servo_angles[7] = 180 - shoulder_2_data.current_angle

        # Elbow
        servo_angles[5] = elbow_data.current_angle
        servo_angles[8] = 180 - elbow_data.current_angle  

        # Writes at 100Hz
        playback_thread = threading.Thread(target=PlaybackRecording, args=(kit, servo_recording))
        if (time.clock() - timestamp) >= 0.01:
            if not playback_thread.is_alive():
                print('writing')
                servo_commands = ServoWrite(kit, servo_angles)
            all_commands = Drive(ABS_Z, ABS_X, left, right, drive_pins, all_commands)
            timestamp = time.clock()
            if record_servos:
                servo_recording.append([servo_commands, timestamp])
            if triangle:
                # playback
                playback_thread.start()
                # PlaybackRecording(kit, servo_recording)
                triangle = False


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

def WindowThresh(signal, threshold):
    if signal >= np.abs(threshold) or signal <= -np.abs(threshold):
        return True
    return False
def PlaybackRecording(servo_kit, servo_recording):
    # servo_states = servo_recording.popleft()
    servo_states = servo_recording[0]
    recording_start_time = servo_states[1]
    current_start_time = time.clock()
    for servo_states in servo_recording:
        print('hey', flush=True)
        # servo_states = servo_recording.popleft()
        time.sleep(servo_states[1] - recording_start_time)
        ServoWrite(servo_kit, servo_states[0])
        recording_start_time = servo_states[1]
    print("done")


if __name__ == '__main__':
    main()
