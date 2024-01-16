import glob
import os
import sys
import math
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

from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
#from agents.tools.misc import draw_waypoints

def draw_waypoints(world, waypoints, z=0.5):
    """
    Draw a list of waypoints at a certain height given in z.

        :param world: carla.world object
        :param waypoints: list or iterable container with the waypoints to draw
        :param z: height in meters
    """
    for wpt in waypoints:
        wpt_t = wpt.transform
        begin = wpt_t.location + carla.Location(z=z)
        angle = math.radians(wpt_t.rotation.yaw)
        end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
        world.debug.draw_arrow(begin, end, arrow_size=0.3, life_time=65.0)

def get_direction(source_waypoint, destination_waypoint, vehicle_transform):
    # Get the vehicle location
    vehicle_location = vehicle_transform.location

    # Calculate the vector from the vehicle to the destination
    destination_vector = carla.Vector3D(
        x=destination_waypoint.transform.location.x - vehicle_location.x,
        y=destination_waypoint.transform.location.y - vehicle_location.y,
        z=destination_waypoint.transform.location.z - vehicle_location.z)

    # Get the forward vector of the vehicle
    forward_vector = vehicle_transform.get_forward_vector()

    # Calculate the dot product and determinant
    dot_product = forward_vector.x * destination_vector.x + forward_vector.y * destination_vector.y
    determinant = forward_vector.x * destination_vector.y - forward_vector.y * destination_vector.x

    # Calculate the angle between the vectors
    angle = math.atan2(determinant, dot_product)

    # Determine the direction
    if abs(angle) < math.pi / 4:
        direction = 'front'
    elif abs(angle) > 3 * math.pi / 4:
        direction = 'back'
    elif angle > 0:
        direction = 'right'
    else:
        direction = 'left'
    print(direction)

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
    vehicle_bp = random.choice(blueprint_library.filter('vehicle'))
    spawn_points = map.get_spawn_points()
    spawn_point = random.choice(spawn_points)
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    destination = carla.Location(x=spawn_point.location.x + 100, y=spawn_point.location.y+50, z=spawn_point.location.z)
    agent = BasicAgent(vehicle)
    agent.set_destination(destination)
    queue = agent.get_local_planner().get_plan()
    waypoints = []
    for waypoint, _ in queue:
        current_waypoint = map.get_waypoint(vehicle.get_location())
        waypoints.append(waypoint)
        get_direction(current_waypoint, waypoint, vehicle.get_transform())
    draw_waypoints(world, waypoints, 1)
    print(f"Vehicle spawned at {spawn_point.location}. Driving to {destination}.")

    pygame.init()
    display = pygame.display.set_mode(
                (1280, 720),
                pygame.HWSURFACE | pygame.DOUBLEBUF)
    display.fill((0,0,0))
    pygame.display.flip()

    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-5, z=2.0),carla.Rotation(yaw=0, pitch=0.0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    camera.listen(lambda image: display.blit(process_image(image), (0, 0)))

    # Create a surface for the minimap
    minimap_surface = pygame.Surface((200, 200))  # Adjust the size as needed

    done = False
    while not done:

        # Draw the minimap
        minimap_position = (1080, 20)  # Position of the minimap on the main display
        minimap_size = (160, 160)  # Size of the minimap on the main display
        draw_minimap(display, minimap_surface, world, vehicle, waypoints, minimap_position, minimap_size)

        pygame.display.flip()


        # Get the current location of the vehicle
        current_location = vehicle.get_location()
        current_waypoint = map.get_waypoint(current_location)

        # Check if there are waypoints left in the queue
        if waypoints:
            # Get the next waypoint
            next_waypoint = waypoints[0]
            # Get the direction to the next waypoint
            get_direction(current_waypoint, next_waypoint, vehicle.get_transform())
            print('================================================================')
            # If the vehicle is close to the next waypoint, remove it from the list
            if current_location.distance(next_waypoint.transform.location) < 2.0:
                waypoints.pop(0)


        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
                elif event.key == pygame.K_s:
                    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0, reverse=True))
                elif event.key == pygame.K_a:
                    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1.0))
                elif event.key == pygame.K_d:
                    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=1.0))
            elif event.type == pygame.KEYUP:
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0))

    camera.destroy()
    vehicle.destroy()
    pygame.quit()

def draw_minimap(display, minimap_surface, world, vehicle, waypoints, position, size):
    # Clear the minimap surface
    minimap_surface.fill((0, 0, 0))

    # Calculate the scale and offset for the minimap based on the vehicle's location and waypoints
    scale, offset = calculate_minimap_scale_and_offset(vehicle, waypoints, size)

    # Draw the blue line for the path
    if waypoints:
        for i in range(len(waypoints) - 1):
            start_pos = world_to_minimap(waypoints[i].transform.location, scale, offset)
            end_pos = world_to_minimap(waypoints[i + 1].transform.location, scale, offset)
            pygame.draw.line(minimap_surface, (0, 0, 255), start_pos, end_pos, 2)

    # Draw the source and destination markers
    source_pos = world_to_minimap(waypoints[0].transform.location, scale, offset) if waypoints else (0, 0)
    destination_pos = world_to_minimap(waypoints[-1].transform.location, scale, offset) if waypoints else (0, 0)
    pygame.draw.circle(minimap_surface, (0, 255, 0), source_pos, 5)
    pygame.draw.circle(minimap_surface, (255, 0, 0), destination_pos, 5)

    # Draw the vehicle's position on the minimap
    vehicle_pos = world_to_minimap(vehicle.get_location(), scale, offset)
    pygame.draw.circle(minimap_surface, (255, 255, 0), vehicle_pos, 5)

    # Scale the minimap to the desired size
    scaled_minimap = pygame.transform.scale(minimap_surface, size)

    # Blit the scaled minimap onto the main display surface at the given position
    display.blit(scaled_minimap, position)

def calculate_minimap_scale_and_offset(vehicle, waypoints, minimap_size):
    # Get the world coordinates of all waypoints and the vehicle's location
    all_points = [vehicle.get_location()] + [waypoint.transform.location for waypoint in waypoints]
    
    # Find the minimum and maximum coordinates
    min_x = min(point.x for point in all_points)
    max_x = max(point.x for point in all_points)
    min_y = min(point.y for point in all_points)
    max_y = max(point.y for point in all_points)
    
    # Calculate the world's width and height based on the waypoints
    world_width = max_x - min_x
    world_height = max_y - min_y
    
    # Calculate the scale factor to fit the world within the minimap
    scale_x = minimap_size[0] / world_width if world_width > 0 else 1
    scale_y = minimap_size[1] / world_height if world_height > 0 else 1
    scale = min(scale_x, scale_y)  # Use the smaller scale to ensure everything fits
    
    # Calculate the offset to center the vehicle on the minimap
    vehicle_pos = vehicle.get_location()
    offset_x = (minimap_size[0] / 2) - (vehicle_pos.x - min_x) * scale
    offset_y = (minimap_size[1] / 2) - (vehicle_pos.y - min_y) * scale
    offset = (offset_x, offset_y)
    
    return scale, offset


def world_to_minimap(location, scale, offset):
    # Convert world coordinates to minimap coordinates using the given scale and offset
    # This function needs to be implemented based on your world's coordinate system
    minimap_x = int(location.x * scale + offset[0])  # Placeholder for actual conversion logic
    minimap_y = int(location.y * scale + offset[1])  # Placeholder for actual conversion logic
    return (minimap_x, minimap_y)

if __name__ == "__main__":
    main()
