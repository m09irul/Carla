import glob
import os
import sys
import math
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
    # Get the vehicle's orientation (yaw angle)
    vehicle_yaw = vehicle_transform.rotation.yaw

    # Calculate the angle between the vehicle's orientation and the direction towards the destination waypoint
    angle_to_destination = math.degrees(math.atan2(destination_waypoint.transform.location.y - vehicle_transform.location.y,
                                                    destination_waypoint.transform.location.x - vehicle_transform.location.x))

    # Calculate the relative angle between the vehicle's orientation and the direction towards the destination
    relative_angle = angle_to_destination - vehicle_yaw

    # Normalize the relative angle to be within the range [-180, 180] degrees
    relative_angle = (relative_angle + 180) % 360 - 180

    # Determine the direction based on the relative angle
    if -45 <= relative_angle <= 45:
        direction = 'front'
    elif -135 <= relative_angle <= -45:
        direction = 'left'
    elif 45 <= relative_angle <= 135:
        direction = 'right'
    else:
        direction = 'back'

    return (relative_angle, direction)

def process_image(image):
    image.convert(carla.ColorConverter.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    surface = pygame.transform.scale(surface, (1280, 720))
    return surface

def get_turn_indexes(waypoint_list):
    indexes = []
    last_found = None
    for i, (_, road_option) in enumerate(waypoint_list):
        print(road_option.name)
        if road_option.name in ['LEFT', 'RIGHT'] and road_option.name != last_found:
            indexes.append((i, road_option.name))  # Store the index and the name of the turn
            last_found = road_option.name
        elif road_option.name not in ['LEFT', 'RIGHT']:
            last_found = None
    return indexes

def calculate_distance(waypoints, current_waypoint, destination_waypoint):
    sum = 0
    prev = current_waypoint.transform.location

    for waypoint in waypoints:
        # Calculate the distance
        distance = prev.distance(waypoint.transform.location)

        #print(f"The distance between the waypoints is {distance} meters.")
        sum += distance
        prev = waypoint.transform.location

        if waypoint == destination_waypoint:
            break
           
    return sum

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
    destination = carla.Location(x=spawn_point.location.x + 6, y=spawn_point.location.y+5, z=spawn_point.location.z)
    agent = BasicAgent(vehicle)
    agent.set_destination(destination)
    queue = agent.get_local_planner().get_plan()
    waypoints = []
    current_waypoint = map.get_waypoint(vehicle.get_location())
        
    print(f"The distance from s to d waypoint is {current_waypoint.transform.location.distance(destination)} meters.")

    # Get the list of waypoints from source to destination

    for waypoint, _ in queue:
        waypoints.append(waypoint)
        
    original_waypoints = waypoints.copy()

    draw_waypoints(world, waypoints, 1)

    turn_indexes = get_turn_indexes(queue)
    # print(f"Vehicle spawned at {spawn_point.location}. Driving to {destination}.")

    pygame.init()
    display = pygame.display.set_mode(
                (1280, 720),
                pygame.HWSURFACE | pygame.DOUBLEBUF)

    pygame.display.flip()

    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-5, z=2.0),carla.Rotation(yaw=0, pitch=0.0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    camera.listen(lambda image: display.blit(process_image(image), (0, 0)))

    done = False
    while not done:

        pygame.display.flip()

        # Get the current location of the vehicle
        current_location = vehicle.get_location()
        current_waypoint = map.get_waypoint(current_location)

        if waypoints:
            # Get the next waypoint
            next_waypoint = waypoints[0]         
            
            if len(turn_indexes) != 0:
                #calculate distance of turns
                dst_wpnt = original_waypoints[turn_indexes[0][0]]
                turn_distance = calculate_distance(waypoints, current_waypoint, dst_wpnt)
                print(f"Turn {turn_indexes[0][1]} in {turn_distance} meters.")

                if current_location.distance(next_waypoint.transform.location) < 4.0:
                    waypoints.pop(0)
                    if next_waypoint == dst_wpnt:
                        turn_indexes.pop(0)


#region keyboard input
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
                elif event.key == pygame.K_SPACE:
                    # Save the image when spacebar is pressed
                    # Generate a unique filename based on the current timestamp
                    timestamp = time.strftime("%Y%m%d%H%M%S")
                    filename = f"image_{timestamp}.png"
                    pygame.image.save(display, filename)
                    print(f"Image saved as {filename}.")
            elif event.type == pygame.KEYUP:
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0))
#endregion
    
    camera.destroy()
    vehicle.destroy()
    pygame.quit()

if __name__ == "__main__":
    main()
