import serial
import pandas as pd
import time

# Configure the serial port
ser = serial.Serial('COM4', 115200)  # Adjust COM port as necessary

# Initialize an empty DataFrame
df = pd.DataFrame(columns=['Acceleration', 'Angle', 'Buzzer'])

try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            data = line.split(', ')
            df = df.append({
                'Acceleration': float(data[0]),
                'Angle': int(data[1]),
                'Buzzer': data[2]
            }, ignore_index=True)
            print(df.tail(1))  # Print the last row for debugging
        time.sleep(0.05)  # Add a small delay for stability
except KeyboardInterrupt:
    # Save the DataFrame to a CSV file when the script is stopped
    df.to_csv('servo_buzzer_log.csv', index=False)
    print('Data saved to servo_buzzer_log.csv')
