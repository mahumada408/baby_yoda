import time
# from tkinter import *
import numpy as np
from adafruit_servokit import ServoKit

def main():

    # gui stuff
#    gui = Tk()
#    gui_list = gui_stuff(gui)

    kit = ServoKit(channels=16)
    counter = 0

    previous_roll_angle = 0
    previous_pitch_angle = 0
    previous_yaw_angle = 0
    previous_arm_angle = 0
    previous_arm_angle_2 = 0

    while True:
        pitch_angle = 0
        roll_angle = 0
        
        # for i in range(len(gui_list)):
            # print(' ', gui_list[i].get(), end = '')
            # if i == 0:
            #     kit.servo[i].angle = gui_list[i].get()
            # if i == 3:
            #     kit.servo[i].angle = gui_list[i].get()
            # if i == 4:
            #     kit.servo[i].angle = gui_list[i].get()
            # if i == 7:
            #     pitch_angle = gui_list[i].get()
            # if i == 8:
            #     roll_angle = gui_list[i].get()

        roll_angle = 10 * np.sin(1 * counter)
        # pitch_angle = 45 * np.sin(counter)
        yaw_angle = 5 * np.sin(counter)

        roll_angle = filter(roll_angle, previous_roll_angle)
        pitch_angle = filter(pitch_angle, previous_pitch_angle)
        yaw_angle = filter(yaw_angle, previous_yaw_angle)

        kit.servo[0].angle = 90 + yaw_angle
        kit.servo[1].angle = 90 - pitch_angle + roll_angle
        kit.servo[2].angle = 90 + pitch_angle + roll_angle

        arm_angle = 45 * np.sin(counter)
        arm_angle_2 = 10 * np.sin(counter)
        arm_angle = filter(arm_angle, previous_arm_angle)
        arm_angle_2 = filter(arm_angle_2, previous_arm_angle_2)
        kit.servo[3].angle = 90 + arm_angle
        # kit.servo[4].angle = 135 + arm_angle
        
        # print()
        # gui.update()
        counter = counter + 0.01

        previous_yaw_angle = yaw_angle
        previous_pitch_angle = pitch_angle
        previous_roll_angle = roll_angle
        previous_arm_angle = arm_angle
        previous_arm_angle_2 = arm_angle_2

def gui_stuff(gui):

    num_servos = 7
    gui_elements = []
    for servo in range(num_servos):
        servo_slider = Scale(gui, label=('servo_' + str(servo)), from_=0, to=180, orient=HORIZONTAL, resolution=0.01)
        servo_slider.pack()
        servo_slider.set(90)
        gui_elements.append(servo_slider)
    
    pitch_slider = Scale(gui, label='pitch', from_=-90, to=90, orient=HORIZONTAL, resolution=0.01)
    pitch_slider.pack()
    pitch_slider.set(0)
    gui_elements.append(pitch_slider)

    roll_slider = Scale(gui, label='roll', from_=-90, to=90, orient=HORIZONTAL, resolution=0.01)
    roll_slider.pack()
    roll_slider.set(0)
    gui_elements.append(roll_slider)

    return gui_elements

# Algorithm from
# https://en.wikipedia.org/wiki/Exponential_smoothing
def filter(current_command, previous_command):
    smoothing_factor = 0.1
    return (smoothing_factor * current_command) + (1 - smoothing_factor) * previous_command

if __name__ == '__main__':
    main()
