import glob
import os
import sys
import math
import pygame
import random
import carla
import numpy as np 

#region path setup
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
#endregion

from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error

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
        direction = 'right'
    elif 45 <= relative_angle <= 135:
        direction = 'left'
    else:
        direction = 'back'

    print(f"Relative Angle: {relative_angle}, Direction: {direction}")

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

    done = False
    while not done:

        pygame.display.flip()

        current_location = vehicle.get_location()
        current_waypoint = map.get_waypoint(current_location)

        if waypoints:
            # Filter waypoints to keep only those ahead of the vehicle
            forward_waypoints = [waypoint for waypoint in waypoints if
                                waypoint.transform.location.x >= current_location.x]
            if forward_waypoints:
                next_waypoint = forward_waypoints[0]
                get_direction(current_waypoint, next_waypoint, vehicle.get_transform())
                print('================================================================')
                if current_location.distance(next_waypoint.transform.location) < 2.0:
                    waypoints.pop(0)

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
            elif event.type == pygame.KEYUP:
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0))
        #endregion
    
    camera.destroy()
    vehicle.destroy()
    pygame.quit()

if __name__ == "__main__":
    main()
