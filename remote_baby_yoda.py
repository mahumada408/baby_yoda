import evdev
import time
from threading import Thread
from queue import Queue
import collections
import numpy as np
from adafruit_servokit import ServoKit

dev = evdev.InputDevice('/dev/input/event2')
events = Queue()
event_deque = collections.deque()

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

    roll_angle = 0
    pitch_angle = 0
    yaw_angle = 0
    arm_angle = 0
    arm_angle_2 = 0

    ABS_X = 0
    ABS_RX = 0
    ABS_RY = 0
    ABS_Z = 90
    ABS_RZ = 0

    previous_roll_angle = 0
    previous_pitch_angle = 0
    previous_yaw_angle = 0
    previous_right_shoulder_angle = 0
    previous_left_shoulder_angle = 90

    previous_roll_time = time.clock()
    previous_pitch_time = previous_roll_time
    previous_yaw_time = previous_pitch_time
    previous_right_shoulder_time = previous_yaw_time
    previous_left_shoulder_time = previous_right_shoulder_time

    num_servos = 9
    servo_angles = [90] * num_servos
    previous_servo_angles = [0] * num_servos

    start_time = time.clock()

    while True:
        # if not events.empty():
        if event_deque:
            # event = events.get_nowait()
            event = event_deque.popleft()
            if 'ABS_X' in str(evdev.categorize(event)):
                ABS_X = np.interp(float(event.value), [0,255], [-45,45])
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

        current_yaw_time = time.clock()
        yaw_angle = clamp_servo_rate(previous_yaw_angle, ABS_X, (current_yaw_time - previous_yaw_time), 100)
        previous_yaw_time = current_yaw_time
        previous_yaw_angle = yaw_angle

        servo_0_angle = 90 + yaw_angle
        servo_angles[0] = servo_0_angle

        current_roll_time = time.clock()
        roll_angle = clamp_servo_rate(previous_roll_angle, ABS_RX, (current_roll_time - previous_roll_time), 100)
        previous_roll_time = current_roll_time
        previous_roll_angle = roll_angle

        current_pitch_time = time.clock()
        pitch_angle = clamp_servo_rate(previous_pitch_angle, ABS_RY, (current_pitch_time - previous_pitch_time), 100)
        previous_pitch_time = current_pitch_time
        previous_pitch_angle = pitch_angle
        pitch_angle = np.clip(pitch_angle, -30, 60)

        servo_1_angle = np.clip(90 - pitch_angle + roll_angle, 0, 180)
        servo_2_angle = np.clip(90 + pitch_angle + roll_angle, 0, 180)

        servo_angles[1] = servo_1_angle
        servo_angles[2] = servo_2_angle

        current_left_shoulder_time = time.clock()
        left_shoulder_angle = clamp_servo_rate(previous_left_shoulder_angle, ABS_Z, (current_left_shoulder_time - previous_left_shoulder_time), 200)
        previous_left_shoulder_time = current_left_shoulder_time
        previous_left_shoulder_angle = left_shoulder_angle

        servo_angles[4] = left_shoulder_angle
        servo_angles[6] = 180 - left_shoulder_angle

        current_right_shoulder_time = time.clock()
        right_shoulder_angle = clamp_servo_rate(previous_right_shoulder_angle, ABS_RZ, (current_right_shoulder_time - previous_right_shoulder_time), 200)
        previous_right_shoulder_time = current_right_shoulder_time
        previous_right_shoulder_angle = right_shoulder_angle

        servo_angles[3] = right_shoulder_angle
        servo_angles[5] = 90 - right_shoulder_angle

        # Writes at 100Hz
        if (time.clock() - start_time) >= 0.01:
            ServoWrite(kit, servo_angles)
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


if __name__ == '__main__':
    main()
