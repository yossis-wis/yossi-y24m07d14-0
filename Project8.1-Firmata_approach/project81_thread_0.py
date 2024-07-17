import time
import sys
import threading
from pymata4 import pymata4

DIGITAL_PIN = 6  # Arduino pin number
LED_PIN = 4

POLL_TIME = 5  # Number of seconds between polls

CB_PIN_MODE = 0
CB_PIN = 1
CB_VALUE = 2
CB_TIME = 3


def the_callback(data):
    """
    A callback function to report data changes.
    This will print the pin number, its reported value and
    the date and time when the change occurred

    :param data: [pin, current reported value, pin_mode, timestamp]
    """
    date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data[CB_TIME]))
    print(f'Pin: {data[CB_PIN]} Value: {data[CB_VALUE]} Time Stamp: {date}')
    board.digital_write(LED_PIN,1)
    time.sleep(1000)
    board.digital_write(LED_PIN,0)


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

    def poll_pin():
        """
        Poll the pin value and print it.
        This function will be called by the timer.
        """
        value, time_stamp = my_board.digital_read(pin)
        date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time_stamp))
        print(f'Polling - last value: {value} received on {date}')
        # Restart the timer
        timer = threading.Timer(POLL_TIME, poll_pin)
        timer.start()
        timers.append(timer)

    # Start the initial timer
    timer = threading.Timer(POLL_TIME, poll_pin)
    timer.start()
    timers.append(timer)


board = pymata4.Pymata4(com_port="COM4")
board.set_pin_mode_digital_output(LED_PIN)

# List to keep track of active timers
timers = []

try:
    digital_in(board, DIGITAL_PIN)
    # Keep the main thread alive to allow timer to function
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    # Shutdown board and cancel timers on interrupt
    board.shutdown()
    for timer in timers:
        timer.cancel()
    sys.exit(0)
