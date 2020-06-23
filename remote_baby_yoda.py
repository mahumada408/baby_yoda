import evdev
import time
from threading import Thread
from queue import Queue
import numpy as np
from adafruit_servokit import ServoKit

dev = evdev.InputDevice('/dev/input/event2')
events = Queue()

def window_threshold(signal, lower, upper):
  if signal >= upper or signal <= lower:
    return True
  return False

def worker():
    for event in dev.read_loop():
      abs_check = window_threshold(event.value, 125, 129)
      if event.type == evdev.ecodes.EV_KEY or abs_check and event.type != evdev.ecodes.SYN_REPORT:
          events.put(event)

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

    previous_roll_angle = 0
    previous_pitch_angle = 0
    previous_yaw_angle = 0
    previous_arm_angle = 0
    previous_arm_angle_2 = 0

    previous_roll_time = time.clock()
    previous_pitch_time = previous_roll_time
    previous_yaw_time = previous_pitch_time

    while True:

        if not events.empty():
            event = events.get_nowait()
            if 'ABS_Y' in str(evdev.categorize(event)):
                print('ABS_Y: ', event.value)
            elif 'ABS_X' in str(evdev.categorize(event)):
                ABS_X = np.interp(float(event.value), [0,255], [-45,45])
            elif 'ABS_RX' in str(evdev.categorize(event)):
                ABS_RX = np.interp(float(event.value), [0,255], [-45,45])
            elif 'ABS_RY' in str(evdev.categorize(event)):
                ABS_RY = -np.interp(float(event.value), [0,255], [-60,60])

        current_yaw_time = time.clock()
        yaw_angle = filter(ABS_X, previous_yaw_angle, 0.1)
        yaw_angle = clamp_servo_rate(previous_yaw_angle, yaw_angle, (current_yaw_time - previous_yaw_time), 100)
        previous_yaw_time = current_yaw_time
        previous_yaw_angle = yaw_angle

        current_roll_time = time.clock()
        roll_angle = filter(ABS_RX, previous_roll_angle, 0.1)
        roll_angle = clamp_servo_rate(previous_roll_angle, roll_angle, (current_roll_time - previous_roll_time), 300)
        previous_roll_time = current_roll_time
        previous_roll_angle = roll_angle

        current_pitch_time = time.clock()
        pitch_angle = filter(ABS_RY, previous_pitch_angle, 0.1)
        pitch_angle = clamp_servo_rate(previous_pitch_angle, pitch_angle, (current_pitch_time - previous_pitch_time), 300)
        previous_pitch_time = current_pitch_time
        previous_pitch_angle = pitch_angle

  
        arm_angle = filter(arm_angle, previous_arm_angle, 0.01)
        arm_angle_2 = filter(arm_angle_2, previous_arm_angle_2, 0.01)

        servo_angle = np.clip(90 - pitch_angle + roll_angle, 0, 180)
        servo_angle_2 = np.clip(90 + pitch_angle + roll_angle, 0, 180)

        kit.servo[0].angle = 90 + yaw_angle
        kit.servo[1].angle = servo_angle
        kit.servo[2].angle = servo_angle_2

        kit.servo[3].angle = 90 + arm_angle
        # kit.servo[4].angle = 135 + arm_angle

        previous_arm_angle = arm_angle
        previous_arm_angle_2 = arm_angle_2

# Algorithm from
# https://en.wikipedia.org/wiki/Exponential_smoothing
def filter(current_command, previous_command, smoothing_factor):
    return (smoothing_factor * current_command) + (1 - smoothing_factor) * previous_command

def clamp_servo_rate(previous_command, current_command, time_dt, rate_limit):
    current_rate = (current_command - previous_command) / time_dt
    print(current_rate)
    if (abs(current_rate) > rate_limit):
        return previous_command + np.sign(current_rate) * rate_limit * time_dt
    else:
        return current_command


if __name__ == '__main__':
    main()
