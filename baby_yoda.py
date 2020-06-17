import time
from tkinter import *

# Import the PCA9685 module.
# import Adafruit_PCA9685


def main():

    # gui stuff
    gui = Tk()
    gui_list = gui_stuff(gui)

    while True:

        for slider in gui_list:
            print(' ', slider.get(), end = '')
        
        print()
        gui.update()

def gui_stuff(gui):

    num_servos = 7
    gui_elements = []
    for servo in range(num_servos):
        servo_slider = Scale(gui, label=('servo_' + str(servo)), from_=-90, to=90, orient=HORIZONTAL, resolution=0.01)
        servo_slider.pack()
        servo_slider.set(0)
        gui_elements.append(servo_slider)

    return gui_elements

if __name__ == '__main__':
    main()
