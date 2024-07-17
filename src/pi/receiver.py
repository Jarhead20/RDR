import serial

# Create a serial object
ser = serial.Serial('/dev/ttyAMA0', 115200)  # Replace '/dev/ttyUSB0' with your serial port

while True:
    # Read data from the serial port
    data = ser.readline().decode().strip()

    # Print the received data
    print(data)