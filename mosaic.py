#!/usr/bin/env python3

import argparse
import time
import subprocess
import cv2
import pygame
import numpy as np
import serial
import sys
import os

# ----------------------------------------------------
# 1) Parse command-line arguments
# ----------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(description="Mosaic scanner with camera and stage control")
    # Serial / Printer options
    parser.add_argument("--serial_port", default="/dev/ttyUSB0", help="Serial port for 3D printer")
    parser.add_argument("--baud_rate", default=115200, type=int, help="Baud rate for printer serial port")

    # Camera device
    parser.add_argument("--camera_device", default="/dev/video4", help="Which /dev/video* device to use")
    parser.add_argument("--frame_width", default=1920, type=int, help="Camera capture width")
    parser.add_argument("--frame_height", default=1080, type=int, help="Camera capture height")
    parser.add_argument("--fps", default=30, type=int, help="Camera capture framerate")

    # v4l2-ctl defaults from your snippet
    parser.add_argument("--brightness", default=83, type=int)
    parser.add_argument("--contrast", default=39, type=int)
    parser.add_argument("--saturation", default=32, type=int)
    parser.add_argument("--white_balance_automatic", default=0, type=int)
    parser.add_argument("--gain", default=6, type=int)
    parser.add_argument("--power_line_frequency", default=1, type=int)
    parser.add_argument("--white_balance_temperature", default=6500, type=int)
    parser.add_argument("--sharpness", default=39, type=int)
    parser.add_argument("--backlight_compensation", default=0, type=int)
    parser.add_argument("--auto_exposure", default=1, type=int)
    parser.add_argument("--exposure_time_absolute", default=47, type=int)
    parser.add_argument("--exposure_dynamic_framerate", default=0, type=int)
    parser.add_argument("--pan_absolute", default=0, type=int)
    parser.add_argument("--tilt_absolute", default=0, type=int)
    parser.add_argument("--focus_absolute", default=0, type=int)
    parser.add_argument("--focus_automatic_continuous", default=0, type=int)
    parser.add_argument("--zoom_absolute", default=1, type=int)

    # Mosaic parameters
    parser.add_argument("--prefix", default="capture/myscan",
                        help="Filename prefix for saved images, e.g. 'capture/myscan' -> 'capture/myscan_0_0.jpeg' etc.")
    parser.add_argument("--x_step", default=10.0, type=float, help="Step size in X direction (mm) for WASD keys")
    parser.add_argument("--y_step", default=10.0, type=float, help="Step size in Y direction (mm) for WASD keys")
    parser.add_argument("--z_step", default=10.0, type=float, help="Step size in Z direction (mm) for Q/E keys")
    parser.add_argument("--n_x", default=5, type=int, help="Number of steps (and images) in X direction for mosaic")
    parser.add_argument("--n_y", default=5, type=int, help="Number of steps (and images) in Y direction for mosaic")
    parser.add_argument("--settle_time", default=500, type=int, 
                        help="Time in milliseconds to wait after a move before capturing image")

    return parser.parse_args()


# ----------------------------------------------------
# 2) Camera control via v4l2-ctl
# ----------------------------------------------------
def set_camera_controls(args):
    """
    Set camera controls using v4l2-ctl. If v4l2-ctl isn't installed or fails,
    this might raise an exception.
    """
    cmd = [
        "v4l2-ctl",
        "-d", args.camera_device,
        "-c", f"brightness={args.brightness}",
        "-c", f"contrast={args.contrast}",
        "-c", f"saturation={args.saturation}",
        "-c", f"white_balance_automatic={args.white_balance_automatic}",
        "-c", f"gain={args.gain}",
        "-c", f"power_line_frequency={args.power_line_frequency}",
        "-c", f"white_balance_temperature={args.white_balance_temperature}",
        "-c", f"sharpness={args.sharpness}",
        "-c", f"backlight_compensation={args.backlight_compensation}",
        "-c", f"auto_exposure={args.auto_exposure}",
        "-c", f"exposure_time_absolute={args.exposure_time_absolute}",
        "-c", f"exposure_dynamic_framerate={args.exposure_dynamic_framerate}",
        "-c", f"pan_absolute={args.pan_absolute}",
        "-c", f"tilt_absolute={args.tilt_absolute}",
        "-c", f"focus_absolute={args.focus_absolute}",
        "-c", f"focus_automatic_continuous={args.focus_automatic_continuous}",
        "-c", f"zoom_absolute={args.zoom_absolute}"
    ]
    print("Setting camera controls:", " ".join(cmd))
    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError:
        print("v4l2-ctl not found. Install v4l-utils or skip if not needed.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to set camera controls: {e}")


# ----------------------------------------------------
# 3) Camera initialization
# ----------------------------------------------------
def init_camera(args):
    """
    Initialize the camera using OpenCV with given width, height, fps, etc.
    Returns cv2.VideoCapture object or None on failure.
    """
    cap = cv2.VideoCapture(args.camera_device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Failed to open camera {args.camera_device}")
        return None

    # Request resolution, FPS, and MJPEG
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.frame_height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    # FourCC for MJPEG
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    # Check if it's actually set
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"Camera opened at resolution: {w}x{h}")

    return cap


# ----------------------------------------------------
# 4) Serial / 3D Printer control
# ----------------------------------------------------
def init_serial(port, baud):
    """
    Open serial connection to the printer.
    """
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect to {port}: {e}")
        return None

def send_gcode(ser, command):
    """
    Send a G-code command to the printer, with a slight delay.
    """
    if ser is not None:
        ser.write(f"{command}\n".encode())
        time.sleep(0.1)

def move_head(ser, dx=0, dy=0, dz=0, speed=1000):
    """
    Move the head relative by dx, dy, dz (in mm).
    Speed in mm/min for all moves.
    We'll switch to G91 (relative), move, then G90 (absolute).
    """
    if ser is None:
        return
    send_gcode(ser, "G91")  # relative mode
    cmd = "G1"
    # Build up the command with whichever axes we have
    if dx != 0:
        cmd += f" X{dx}"
    if dy != 0:
        cmd += f" Y{dy}"
    if dz != 0:
        cmd += f" Z{dz}"
    cmd += f" F{speed}"
    send_gcode(ser, cmd)
    send_gcode(ser, "G90")  # back to absolute mode


# ----------------------------------------------------
# 5) Mosaic scanning routine
# ----------------------------------------------------
def run_mosaic(ser, cap, args):
    """
    Performs a grid (n_x by n_y) capture. For each grid cell:
      - Move to the new position (relative moves)
      - Wait settle_time ms
      - Capture frame
      - Save as prefix_x_y.jpeg (80% quality)
    We'll do a simple "raster" approach:
      For y in [0..n_y-1]:
        For x in [0..n_x-1]:
          capture/save
          if x < n_x-1: move in X
        if y < n_y-1:
          move X back, move Y up/down
    """
    prefix = args.prefix
    x_step = args.x_step
    y_step = args.y_step
    n_x    = args.n_x
    n_y    = args.n_y
    settle = args.settle_time / 1000.0  # convert ms -> sec

    # Let user confirm they're at the "origin" they'd like as (0,0).
    print("Starting mosaic capture. Make sure you're at the desired (0,0).")

    # We'll do relative moves:
    for row in range(n_y):
        for col in range(n_x):
            # Wait for settle
            time.sleep(settle)

            # Capture
            print('cap')
            for _ in range(5): # 5 frames to let things settle more
                ret, frame = cap.read()
            if not ret:
                print("Failed to capture image at position", (col, row))
                continue
            # Save image
            filename = f"{prefix}_{col}_{row}.jpeg"
            # Encode with 80% quality
            cv2.imwrite(filename, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            print("Saved:", filename)

            # Move in X if not last column
            if col < n_x - 1:
                move_head(ser, dx=x_step, dy=0, dz=0)

        # After finishing a row, move back in X and then move in Y if not last row
        if row < n_y - 1:
            # Move back in X by (n_x-1)*x_step
            move_head(ser, dx=-x_step*(n_x-1), dy=0, dz=0)
            # Move Y
            move_head(ser, dx=0, dy=y_step, dz=0)

    print("Mosaic complete!")


# ----------------------------------------------------
# 6) Main Pygame loop
# ----------------------------------------------------
def main():
    args = parse_args()

    # Attempt to set camera controls via v4l2-ctl
    set_camera_controls(args)

    # Init camera
    cap = init_camera(args)
    if cap is None:
        print("Camera not available; exiting.")
        sys.exit(1)

    # Init serial
    ser = init_serial(args.serial_port, args.baud_rate)
    if ser is None:
        print("Printer not available; exiting.")
        sys.exit(1)

    # Init Pygame
    pygame.init()
    screen_width, screen_height = 1920, 1080
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Mosaic Control")
    font = pygame.font.SysFont("Arial", 18)

    # Basic usage instructions
    info_text = [
        "W/A/S/D = Move X/Y by x_step, Q/E = Move Z by z_step",
        "I/J/K/L = Move X/Y by x_step/10, T/F/G/H = Move X/Y by x_step*10",
        "Space = Start mosaic scan   Esc = Quit"
    ]

    clock = pygame.time.Clock()

    running = True
    while running:
        # 6.1) Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                key = event.key

                # Quit on ESC
                if key == pygame.K_ESCAPE:
                    running = False

                # Movement for XY (normal step = x_step, y_step)
                elif key == pygame.K_w:  # forward +Y
                    move_head(ser, dx=0, dy=args.y_step)
                elif key == pygame.K_s:  # backward -Y
                    move_head(ser, dx=0, dy=-args.y_step)
                elif key == pygame.K_a:  # left -X
                    move_head(ser, dx=-args.x_step, dy=0)
                elif key == pygame.K_d:  # right +X
                    move_head(ser, dx=args.x_step, dy=0)

                # Movement for XY (10x bigger: T/F/G/H)
                elif key == pygame.K_t:  # forward +Y *10
                    move_head(ser, dx=0, dy=args.y_step*10)
                elif key == pygame.K_g:  # backward -Y *10
                    move_head(ser, dx=0, dy=-args.y_step*10)
                elif key == pygame.K_f:  # left -X *10
                    move_head(ser, dx=-args.x_step*10, dy=0)
                elif key == pygame.K_h:  # right +X *10
                    move_head(ser, dx=args.x_step*10, dy=0)

                # Movement for XY (1/10 smaller: I/J/K/L)
                elif key == pygame.K_i:  # forward +Y /10
                    move_head(ser, dx=0, dy=args.y_step*0.1)
                elif key == pygame.K_k:  # backward -Y /10
                    move_head(ser, dx=0, dy=-args.y_step*0.1)
                elif key == pygame.K_j:  # left -X /10
                    move_head(ser, dx=-args.x_step*0.1, dy=0)
                elif key == pygame.K_l:  # right +X /10
                    move_head(ser, dx=args.x_step*0.1, dy=0)

                # Q/E for Z
                elif key == pygame.K_q:
                    move_head(ser, dz=args.z_step)
                elif key == pygame.K_e:
                    move_head(ser, dz=-args.z_step)

                # Start mosaic capture
                elif key == pygame.K_SPACE:
                    print("Starting mosaic scanning...")
                    run_mosaic(ser, cap, args)
                    print("Mosaic scanning finished.")

        # 6.2) Capture current camera frame for live preview
        ret, frame = cap.read()
        if ret:
            # Convert BGR -> RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # Optionally downscale for display if camera is large
            disp_height = 450
            disp_width = int(frame_rgb.shape[1] * (disp_height / frame_rgb.shape[0]))
            frame_small = cv2.resize(frame_rgb, (disp_width, disp_height), interpolation=cv2.INTER_AREA)

            # Convert to pygame surface
            surface = pygame.surfarray.make_surface(np.rot90(frame_small))
        else:
            # If we can't read, just fill black
            surface = pygame.Surface((640, 480))
            surface.fill((0,0,0))

        # 6.3) Draw everything
        screen.fill((0, 0, 0))
        # Blit camera feed
        screen.blit(surface, (10, 10))

        # Draw instructions
        y_offset = 10
        for line in info_text:
            text_surf = font.render(line, True, (255, 255, 255))
            screen.blit(text_surf, (disp_width + 20, y_offset))
            y_offset += 25

        pygame.display.flip()
        clock.tick(30)  # limit to ~30 fps in the loop

    # Cleanup
    if cap:
        cap.release()
    if ser:
        ser.close()
    pygame.quit()

# ----------------------------------------------------
# 7) Entry point
# ----------------------------------------------------
if __name__ == "__main__":
    main()
