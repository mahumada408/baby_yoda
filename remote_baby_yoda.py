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

ServoData = collections.namedtuple("ServoData", "current_angle previous_time rate_lim")

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

        yaw_data = CleanAngle(ABS_X, yaw_data.current_angle, yaw_data.previous_time, yaw_data.rate_lim)
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


if __name__ == '__main__':
    main()
