import evdev
from time import sleep
from threading import Thread
from queue import Queue

dev = evdev.InputDevice('/dev/input/event8')
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

t = Thread(target=worker)
t.start()

while 42:
    if not events.empty():
        event = events.get_nowait()
        # print(evdev.categorize(event))
        if 'ABS_Z' in str(evdev.categorize(event)):
          print('ABSY_Y: ', event.value)
        elif 'ABS_RZ' in str(evdev.categorize(event)):
          print('ABSY_X: ', event.value)
        # elif 'ABS_X' in str(evdev.categorize(event)):
        #   print('ABSY_X: ', event.value)
        # elif 'ABS_RX' in str(evdev.categorize(event)):
        #   print('ABSY_RX: ', event.value)
        # elif 'ABS_RY' in str(evdev.categorize(event)):
        #   print('ABSY_RY: ', event.value)

    # print('looping')
    # sleep(0.1)