import pygame
import serial
import sys
import time

# Set up serial communication
try:
    ser = serial.Serial('COM3', 115200, timeout=1)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    sys.exit()

# Initialize pygame
pygame.init()
pygame.joystick.init()

# Check for available joysticks
if pygame.joystick.get_count() == 0:
    print("No joystick found.")
    pygame.quit()
    sys.exit()

# Initialize the combined joystick
joystick = pygame.joystick.Joystick(0)  # Assume the joystick is combined
joystick.init()

print(f"Joystick: {joystick.get_name()} (Y-axis and Z-axis)")

# Function to map joystick value to motor command (0 to 255)
def map_axis_to_motor_command(axis_value):
    max_speed = 255
    command = int((1 + axis_value) * max_speed / 2)  # Remap from [-1, 1] to [0, 255]
    command = min(max(command, 0), max_speed)  # Clamp the command to the valid range
    return command

# Main loop
try:
    while True:
        # Process events
        pygame.event.pump()

        # Get Y-axis value from the left joystick (axis 1)
        y_value = joystick.get_axis(1)  # Left joystick Y-axis
        
        # Get Z-axis value from the right joystick (treated as Z-axis on the same joystick, axis 2)
        z_value = joystick.get_axis(2)  # Right joystick Z-axis

        # Map joystick values to motor commands
        motor_command_y = map_axis_to_motor_command(y_value)
        motor_command_z = map_axis_to_motor_command(z_value)

        # Variables for separate motor commands
        motor_y_axis = motor_command_y   # Motor variable for Y-axis (0-255)
        motor_z_axis = motor_command_z   # Motor variable for Z-axis (0-255)

        # Send motor commands to ESP32 with unique IDs
        #ser.write(bytes([0x01, motor_y_axis]))  # 0x01 as ID for Y-axis motor command
        #ser.write(bytes([0x02, motor_z_axis]))  # 0x02 as ID for Z-axis motor command
        ser.write(bytes([0x01, motor_y_axis, 0x02, motor_z_axis]))  # Send both commands together

        # Debug output
        print(f"Y-axis motor command: {motor_y_axis}, Z-axis motor command: {motor_z_axis}")
        
        time.sleep(0.05)  # Adjust this to control polling rate

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    pygame.quit()
