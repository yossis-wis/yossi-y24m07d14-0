import serial
import PySimpleGUI as sg
import threading
import time

# Define the serial port settings (update with your port and baud rate)
SERIAL_PORT = 'COM4' 
BAUD_RATE = 9600  # Match the baud rate used by your device

# Create a serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# PySimpleGUI layout
layout = [
    [sg.Text('Choose a number of seconds for the LED:'), sg.InputText(key='-IN-')],
    [sg.Button('Send')],
    [sg.Output(size=(60, 20))],  # Output area to display messages (optional)
    [sg.Button('Exit')]
]

# Create the window
window = sg.Window('Serial Communication', layout, finalize=False)

# Function to continuously read from serial port
def read_serial():
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            print(f'Received from Arduino: {response}')


#Create a thread to read from serial port
thread = threading.Thread(target=read_serial)
thread.daemon = True  # Set as daemon thread
thread.start()

# Event loop
while True:
        # --------- Read and update window --------
    event, values = window.read()

    if event == sg.WINDOW_CLOSED or event == 'Exit':
        break

    if event == 'Send':
        command = values['-IN-']
        if command:
            # Send command to Arduino via serial
            ser.write((command + '\n').encode('utf-8'))
            print(f'Sent command to Arduino: {command}')

# Close the serial connection and GUI window
ser.close()
window.close()