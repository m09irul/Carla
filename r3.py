# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
import glob
import os
import sys
import math

import pygame

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
from agents.tools.misc import draw_waypoints
import carla
import random


def get_direction(current_waypoint, next_waypoint):
    # Assume 'current_waypoint' and 'next_waypoint' are defined
    current_transform = current_waypoint.transform
    next_transform = next_waypoint.transform

    # Calculate direction vectors
    current_dir = current_transform.get_forward_vector()
    next_dir = next_transform.get_forward_vector()

    # Calculate the angle between the vectors
    angle = math.degrees(math.atan2(current_dir.y, current_dir.x) - math.atan2(next_dir.y, next_dir.x))

    # Normalize the angle to [-180, 180]
    angle = (angle + 180) % 360 - 180

    # Determine the direction
    if abs(angle) < 15:  # adjust the value as needed
        direction = "straight"
    elif angle < 0:
        direction = "right"
    else:
        direction = "left"

    print(direction)

def process_gnss(gnss_data):
    global vehicle
    global agent
    global world

    vehicle_location = gnss_data.transform.location
    map = world.get_map()
    nearest_waypoint = map.get_waypoint(vehicle_location)
    agent.set_destination(nearest_waypoint.transform.location)

    queue = agent.get_local_planner().get_plan()
    waypoints = []
    for waypoint, _ in queue:
            current_waypoint = map.get_waypoint(vehicle.get_location())
            waypoints.append(waypoint)
            get_direction(current_waypoint, waypoint)
            #print(waypoint.transform.location)
    draw_waypoints(vehicle.get_world(), waypoints, 10)

def process_events(vehicle):
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:  # Forward
                vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
            elif event.key == pygame.K_DOWN:  # Backward
                vehicle.apply_control(carla.VehicleControl(throttle=1.0, reverse=True))
            elif event.key == pygame.K_LEFT:  # Left
                vehicle.apply_control(carla.VehicleControl(steer=-1.0))
            elif event.key == pygame.K_RIGHT:  # Right
                vehicle.apply_control(carla.VehicleControl(steer=1.0))
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                vehicle.apply_control(carla.VehicleControl(throttle=0.0))
            elif event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                vehicle.apply_control(carla.VehicleControl(steer=0.0))

def main():
    global vehicle
    global agent
    global world

    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    #client.reload_world()

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

    gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')
    gnss_location = carla.Location(x=1, y=0, z=2)
    gnss_rotation = carla.Rotation()
    gnss_transform = carla.Transform(gnss_location, gnss_rotation)
    gnss_sensor = world.spawn_actor(gnss_bp, gnss_transform, attach_to=vehicle)
    gnss_sensor.listen(lambda data: process_gnss(data))

    print(f"Vehicle spawned at {spawn_point.location}. Driving to {destination}.")

    try:
        pygame.init()
        display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        while True:
            process_events(vehicle)

    finally:
        pygame.quit()

if __name__ == "__main__":
    main()
