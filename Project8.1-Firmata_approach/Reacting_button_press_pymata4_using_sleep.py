import time
import sys
from pymata4 import pymata4

board = pymata4.Pymata4(com_port="COM4")

DIGITAL_PIN = 6  # Arduino pin number
LED_PIN = 4

POLL_TIME = 5  # Number of seconds between polls

CB_PIN_MODE = 0
CB_PIN = 1
CB_VALUE = 2
CB_TIME = 3


# board.set_pin_mode_digital_output(LED_PIN)

def the_callback(data):
    """
    A callback function to report data changes.
    This will print the pin number, its reported value and
    the date and time when the change occurred

    :param data: [pin, current reported value, pin_mode, timestamp]
    """
    date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data[CB_TIME]))
    print(f'Pin: {data[CB_PIN]} Value: {data[CB_VALUE]} Time Stamp: {date}')

def digital_in(my_board, pin):
    """
    This function establishes the pin as a
    digital input. Any changes on this pin will
    be reported through the call back function.

    :param my_board: a pymata_express instance
    :param pin: Arduino pin number
    """

    # Set the pin mode
    my_board.set_pin_mode_digital_input(pin, callback=the_callback)

    while True:
        try:
            # Do a read of the last value reported every 5 seconds and print it
            value, time_stamp = my_board.digital_read(pin)
            date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time_stamp))
            print(f'Polling - last value: {value} received on {date}')
            time.sleep(POLL_TIME)
        except KeyboardInterrupt:
            board.shutdown()
            sys.exit(0)

# board = pymata4.Pymata4()
# board.digital_write(LED_PIN,1)

try:
    digital_in(board, DIGITAL_PIN)
except KeyboardInterrupt:
    board.shutdown()
    sys.exit(0)

board.shutdown()