import glob
import os
import sys
import time
import pygame
import random
import carla
import numpy as np 

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/carla')
except IndexError:
    pass

def process_image(image):
    image.convert(carla.ColorConverter.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    surface = pygame.transform.scale(surface, (1280, 720))
    return surface

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    world = client.get_world()
    map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('model3')[0]
    spawn_points = map.get_spawn_points()
    spawn_point = random.choice(spawn_points)
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    pygame.init()
    # Initialize the joystick module
    pygame.joystick.init()

    # Check if there are any joysticks
    if pygame.joystick.get_count() > 0:
        # Use the first joystick
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
    else:
        print("No joystick found.")
        sys.exit()

    # Initialize control variables
    steering = 0
    throttle = 0
    brake = 0

    display = pygame.display.set_mode(
                (1280, 720),
                pygame.HWSURFACE | pygame.DOUBLEBUF)

    pygame.display.flip()

    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute(f'image_size_x', f'{1920/1.5}')
    camera_bp.set_attribute(f'image_size_y', f'{1080/1.5}')
    camera_bp.set_attribute('fov', '110')
    camera_transform = carla.Transform(carla.Location(x=0, z=2.0),carla.Rotation(yaw=0, pitch=0.0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    camera.listen(lambda image: display.blit(process_image(image), (0, 0)))

    done = reverse = False

    # Initialize control variables
    steering = 0
    throttle = 0
    brake = 0
    reverse = False

    # Smoothing factors
    steering_smooth = 0.5
    throttle_smooth = 0.5
    brake_smooth = 0.5

    # Event loop
    done = False
    while not done:
        pygame.display.flip()

        #region keyboard input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
            # Joystick event handling
            if event.type == pygame.JOYAXISMOTION:
                # Axis motion is a joystick event
                # event.axis is the index of the axis that moved
                # event.value is the new value of the axis (-1 to 1)
                if event.axis == 0:
                    steering = (1 - steering_smooth) * steering + steering_smooth * event.value
                elif event.axis == 1:
                    throttle = (1 - throttle_smooth) * throttle + throttle_smooth * max(0, -event.value)  # Adjust for acceleration pedal
                    brake = (1 - brake_smooth) * brake + brake_smooth * max(0, event.value)  # Adjust for brake pedal

            # Add a joystick button event to toggle reverse
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 10:  # Replace with the correct button index
                    reverse = True
                elif event.button == 11:  # Replace with the correct button index
                    reverse = False

        # Apply control to the vehicle
        control = carla.VehicleControl()
        control.reverse = reverse
        control.steer = steering
        control.throttle = throttle
        control.brake = brake
        print(control) 
        vehicle.apply_control(control)

        # Add a small delay to smooth the control inputs
        time.sleep(0.01)

#endregion
    
    camera.destroy()
    vehicle.destroy()
    pygame.quit()

if __name__ == "__main__":
    main()
