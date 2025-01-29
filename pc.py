import pygame
import serial
import time

# Constants
SERIAL_PORT = "/dev/ttyUSB0"  # Change this to your printer's serial port
BAUD_RATE = 115200
MOVE_DISTANCE = 10  # Distance to move in mm
MOVE_SPEED = 1000   # Speed in mm/min

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("3D Printer Control")
font = pygame.font.SysFont("Arial", 18)

# Initialize Serial Connection
def init_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect to {SERIAL_PORT}: {e}")
        return None

# Send G-code command to the printer
def send_gcode(ser, command):
    if ser is not None:
        ser.write(f"{command}\n".encode())
        time.sleep(0.1)  # Wait for the command to be processed

# Move the printer head
def move_head(ser, direction):
    if ser is None:
        return

    if direction == "up":
        send_gcode(ser, f"G91\nG1 Z{MOVE_DISTANCE} F{MOVE_SPEED}\nG90")
    elif direction == "down":
        send_gcode(ser, f"G91\nG1 Z-{MOVE_DISTANCE} F{MOVE_SPEED}\nG90")
    elif direction == "left":
        send_gcode(ser, f"G91\nG1 X-{MOVE_DISTANCE} F{MOVE_SPEED}\nG90")
    elif direction == "right":
        send_gcode(ser, f"G91\nG1 X{MOVE_DISTANCE} F{MOVE_SPEED}\nG90")
    elif direction == "forward":
        send_gcode(ser, f"G91\nG1 Y{MOVE_DISTANCE} F{MOVE_SPEED}\nG90")
    elif direction == "backward":
        send_gcode(ser, f"G91\nG1 Y-{MOVE_DISTANCE} F{MOVE_SPEED}\nG90")

# Draw the GUI
def draw_gui():
    screen.fill((0, 0, 0))
    text = font.render("Use WASD to move XY, Q/E to move Z", True, (255, 255, 255))
    screen.blit(text, (20, 20))
    pygame.display.flip()

# Main loop
def main():
    ser = init_serial()
    if ser is None:
        return

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    move_head(ser, "forward")
                elif event.key == pygame.K_s:
                    move_head(ser, "backward")
                elif event.key == pygame.K_a:
                    move_head(ser, "left")
                elif event.key == pygame.K_d:
                    move_head(ser, "right")
                elif event.key == pygame.K_q:
                    move_head(ser, "up")
                elif event.key == pygame.K_e:
                    move_head(ser, "down")

        draw_gui()

    ser.close()
    pygame.quit()

if __name__ == "__main__":
    main()