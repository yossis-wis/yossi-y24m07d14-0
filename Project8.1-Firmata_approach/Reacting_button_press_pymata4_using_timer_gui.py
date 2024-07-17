import time
import sys
import threading
from pymata4 import pymata4
import PySimpleGUI as sg

DIGITAL_PIN = 6  # Arduino pin number
LED_PIN = 4

POLL_TIME = .2  # Number of seconds between polls
OFF_TIME = 5  # Number of seconds between polls

CB_PIN_MODE = 0
CB_PIN = 1
CB_VALUE = 2
CB_TIME = 3

global led0

# Define the layout for the window
layout = [
    [sg.Text('', size=(20, 1), key='-TEXT-')],  # Text element to display messages
    [sg.Button('Write Hello'), sg.Button('Exit')]  # Buttons to trigger actions
]

# Create the window
window = sg.Window('Hello Window', layout)


def turnitoff():
    board.digital_write(LED_PIN,0)

def the_callback(data):
    """
    A callback function to report data changes.
    This will print the pin number, its reported value and
    the date and time when the change occurred

    :param data: [pin, current reported value, pin_mode, timestamp]
    """
    global led0
    led0 = data[CB_VALUE]

    date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data[CB_TIME]))
    print(f'Pin: {data[CB_PIN]} Value: {data[CB_VALUE]} Time Stamp: {date}')
    if data[CB_VALUE]:
        board.digital_write(LED_PIN,1)
        # Start the initial timer
        timer = threading.Timer(OFF_TIME, turnitoff)
        timer.start()


def digital_in(my_board, pin):
    """
     This function establishes the pin as a
     digital input. Any changes on this pin will
     be reported through the call back function.

     :param my_board: a pymata_express instance
     :param pin: Arduino pin number
     """

    global led0

    # set the pin mode
    my_board.set_pin_mode_digital_input(pin, callback=the_callback)

    while True:
        try:
            # Do a read of the last value reported every 5 seconds and print it
            # digital_read returns A tuple of last value change and the time that it occurred
            value, time_stamp = my_board.digital_read(pin)
            date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time_stamp))
            # value
            print(f'Polling - last value: {value} received on {date} ')

            event, values = window.read()
            if event == sg.WIN_CLOSED or event == 'Exit':  # If user closes window or clicks Exit
                break
            if event == 'Write Hello':
                # add text that led on performed in response to the button press
                window['-TEXT-'].update(led0)  # Write "Hello" to the text element
            time.sleep(POLL_TIME)

        except KeyboardInterrupt:
            board.shutdown()
            window.close()
            sys.exit(0)
            

board = pymata4.Pymata4(com_port="COM4")
board.set_pin_mode_digital_output(LED_PIN)

try:
    digital_in(board, DIGITAL_PIN)
    # Keep the main thread alive to allow timer to function
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    # Shutdown board and cancel timers on interrupt
    board.shutdown()
    window.close()
    sys.exit(0)
    
