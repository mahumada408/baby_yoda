#!/usr/bin/env python
import serial

ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)

while True :
  state = ""
  state=ser.readline().decode('utf-8')
  if "c" in state:
    print(state, end='')