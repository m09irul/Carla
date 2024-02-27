import pygame
import sys

pygame.init()

# Initialize joystick
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)  # Assume the joystick is at index 0
    joystick.init()
else:
    print("Please connect a joystick and run again.")
    sys.exit(0)

try:
    while True:
        pygame.event.pump()  # Process event queue

        # Iterate over all axes and print their values
        for i in range(joystick.get_numaxes()):
            axis_value = joystick.get_axis(i)
            print(f"Axis {i} value: {axis_value}")

        pygame.time.wait(10)  # Add a small delay to make the output readable

except KeyboardInterrupt:
    print("Exiting...")
    pygame.quit()
