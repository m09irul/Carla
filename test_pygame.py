import random
import carla
import pygame
import numpy as np 

# Connect to the Carla server
client = carla.Client('localhost', 2000)
client.set_timeout(20.0)

world = client.get_world()
map = world.get_map()

# Initialize Pygame and create a window
pygame.init()
display = pygame.display.set_mode(
            (1280, 720),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
display.fill((0,0,0))
pygame.display.flip()

# Get a vehicle blueprint
vehicle_bp = world.get_blueprint_library().filter('vehicle.*')[0]

# Set up the vehicle transform
vehicle_loc = carla.Location(x=-46.9, y=20.0, z=0.2)
vehicle_rot = carla.Rotation(pitch=0.0, yaw=142.0, roll=0.0)
vehicle_trans = carla.Transform(vehicle_loc, vehicle_rot)

# Spawn the vehicle
my_vehicle = world.spawn_actor(vehicle_bp, vehicle_trans)

# Create a camera and attach it to the vehicle
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_transform = carla.Transform(carla.Location(x=-5, z=2.0),carla.Rotation(yaw=0, pitch=0.0))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=my_vehicle)

# Create a function to convert images from the camera to Pygame surfaces
def process_image(image):
    image.convert(carla.ColorConverter.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    # Scale the surface to fill the display
    surface = pygame.transform.scale(surface, (1280, 720))
    return surface


# Listen for images from the camera
camera.listen(lambda image: display.blit(process_image(image), (0, 0)))

# Game loop
done = False
while not done:
    pygame.display.flip()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                # Move forward
                my_vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
            elif event.key == pygame.K_s:
                # Move backward
                my_vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0, reverse=True))
            elif event.key == pygame.K_a:
                # Turn left
                my_vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1.0))
            elif event.key == pygame.K_d:
                # Turn right
                my_vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=1.0))
        elif event.type == pygame.KEYUP:
            # Stop moving when the key is released
            my_vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0))


# Clean up
camera.destroy()
my_vehicle.destroy()
pygame.quit()
