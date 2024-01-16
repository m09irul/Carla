import glob
import os
import sys
import math
import pygame
import random
import carla
import numpy as np

# Add Carla Python API to the sys path
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# Add Carla Python API to the sys path using the relative path
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/carla')
except IndexError:
    pass

from agents.navigation.basic_agent import BasicAgent

class CarlaSimulation:
    def __init__(self):
        # Initialize the Carla client
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(20.0)
        
        # Get the current Carla world, map, and blueprint library
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.blueprint_library = self.world.get_blueprint_library()

        # Choose a random vehicle blueprint and spawn point
        self.vehicle_bp = random.choice(self.blueprint_library.filter('vehicle'))
        self.spawn_points = self.map.get_spawn_points()
        self.spawn_point = random.choice(self.spawn_points)

        # Initialize instance variables
        self.vehicle = None
        self.destination = carla.Location(x=self.spawn_point.location.x + 100, y=self.spawn_point.location.y + 50,
                                          z=self.spawn_point.location.z)
        self.agent = None
        self.queue = None
        self.waypoints = []

        self.printed_waypoints = []  # Maintain a list of printed waypoints

    def setup(self):
        # Spawn the vehicle and set up the agent
        self.vehicle = self.world.spawn_actor(self.vehicle_bp, self.spawn_point)
        self.agent = BasicAgent(self.vehicle)
        self.agent.set_destination(self.destination)
        self.queue = self.agent.get_local_planner().get_plan()

        # Generate waypoints and print initialization information
        for waypoint, _ in self.queue:
            current_waypoint = self.map.get_waypoint(self.vehicle.get_location())
            self.waypoints.append(waypoint)
            self.get_direction(current_waypoint, waypoint, self.vehicle.get_transform())
        self.draw_waypoints(1)
        print(f"Vehicle spawned at {self.spawn_point.location}. Driving to {self.destination}.")

    def run(self):
        # Initialize Pygame
        pygame.init()
        display = pygame.display.set_mode((1280, 720), pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0, 0, 0))
        pygame.display.flip()

        # Spawn a camera for visualization
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=-5, z=2.0), carla.Rotation(yaw=0, pitch=0.0))
        camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        camera.listen(lambda image: display.blit(self.process_image(image), (0, 0)))

        # Create a minimap surface for visualization
        minimap_surface = pygame.Surface((200, 200))

        done = False
        while not done:
            self.draw_minimap(display, minimap_surface)
            pygame.display.flip()

            # Check waypoints and handle Pygame events
            self.check_waypoints()
            done = self.handle_pygame_events()

        # Cleanup resources
        camera.destroy()
        self.vehicle.destroy()
        pygame.quit()

    def draw_waypoints(self, z=0.5):
        # Draw waypoints on the Carla world for visualization
        for wpt in self.waypoints:
            wpt_t = wpt.transform
            begin = wpt_t.location + carla.Location(z=z)
            angle = math.radians(wpt_t.rotation.yaw)
            end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
            self.world.debug.draw_arrow(begin, end, arrow_size=0.3, life_time=65.0)

    def check_waypoints(self):
        # Check if the vehicle has reached a waypoint and update the waypoint list
        current_location = self.vehicle.get_location()
        current_waypoint = self.map.get_waypoint(current_location)

        if self.waypoints:
            next_waypoint = self.waypoints[0]
            self.get_direction(current_waypoint, next_waypoint, self.vehicle.get_transform())
            if current_location.distance(next_waypoint.transform.location) < 2.0:
                self.waypoints.pop(0)

            # Print nearby waypoints under 100 meters
            self.print_nearby_waypoints(current_location, self.waypoints, threshold_distance=10.0)

    def print_nearby_waypoints(self, current_location, waypoints, threshold_distance):
        # Print waypoints and their directions that are under the threshold distance from the current location
        nearby_waypoints = []

        for waypoint in waypoints:
            distance = current_location.distance(waypoint.transform.location)
            if distance < threshold_distance:
                nearby_waypoints.append((waypoint.transform.location, distance, waypoint))

        # Compare with the previously printed waypoints
        if nearby_waypoints != self.printed_waypoints:
            print("\n")
            print("="*100)
            print("Nearby waypoints under 100 meters:")
            for waypoint_location, distance, waypoint in nearby_waypoints:
                print(f"Waypoint at {waypoint_location} is {distance:.2f} meters away.")
                direction = self.get_direction(
                    self.map.get_waypoint(current_location),
                    waypoint,
                    self.vehicle.get_transform()
                )
                print(f"Direction: {direction}")

            # Update the printed waypoints list
            self.printed_waypoints = nearby_waypoints

    def handle_pygame_events(self):
        # Handle Pygame events for controlling the vehicle
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYDOWN:
                self.handle_keydown_event(event)
            elif event.type == pygame.KEYUP:
                self.handle_keyup_event()

    def handle_keydown_event(self, event):
        # Handle keydown events for controlling the vehicle
        if event.key == pygame.K_w:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
        elif event.key == pygame.K_s:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0, reverse=True))
        elif event.key == pygame.K_a:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1.0))
        elif event.key == pygame.K_d:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=1.0))

    def handle_keyup_event(self):
        # Handle keyup events to stop the vehicle
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0))

    @staticmethod
    def get_direction(source_waypoint, destination_waypoint, vehicle_transform):
        # Calculate and print the direction of the vehicle
        # (front, back, right, or left) based on waypoints
        vehicle_location = vehicle_transform.location
        destination_vector = carla.Vector3D(
            x=destination_waypoint.transform.location.x - vehicle_location.x,
            y=destination_waypoint.transform.location.y - vehicle_location.y,
            z=destination_waypoint.transform.location.z - vehicle_location.z)

        forward_vector = vehicle_transform.get_forward_vector()

        dot_product = forward_vector.x * destination_vector.x + forward_vector.y * destination_vector.y
        determinant = forward_vector.x * destination_vector.y - forward_vector.y * destination_vector.x

        angle = math.atan2(determinant, dot_product)

        if abs(angle) < math.pi / 4:
            direction = 'front'
        elif abs(angle) > 3 * math.pi / 4:
            direction = 'back'
        elif angle > 0:
            direction = 'right'
        else:
            direction = 'left'
        #print(direction)
        return direction

    @staticmethod
    def process_image(image):
        # Process the camera image for Pygame display
        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        surface = pygame.transform.scale(surface, (1280, 720))
        return surface

    def draw_minimap(self, display, minimap_surface):
        # Draw the minimap for visualization
        minimap_surface.fill((0, 0, 0))

        # Calculate minimap scale and offset
        scale, offset = self.calculate_minimap_scale_and_offset(self.vehicle, self.waypoints, (160, 160))

        if self.waypoints:
            # Draw waypoints on the minimap
            for i in range(len(self.waypoints) - 1):
                start_pos = self.world_to_minimap(self.waypoints[i].transform.location, scale, offset)
                end_pos = self.world_to_minimap(self.waypoints[i + 1].transform.location, scale, offset)
                pygame.draw.line(minimap_surface, (0, 0, 255), start_pos, end_pos, 2)

        # Draw source and destination circles on the minimap
        source_pos = self.world_to_minimap(self.waypoints[0].transform.location, scale, offset) if self.waypoints else (0, 0)
        destination_pos = self.world_to_minimap(self.waypoints[-1].transform.location, scale, offset) if self.waypoints else (0, 0)
        pygame.draw.circle(minimap_surface, (0, 255, 0), source_pos, 5)
        pygame.draw.circle(minimap_surface, (255, 0, 0), destination_pos, 5)

        # Draw the vehicle circle on the minimap
        vehicle_pos = self.world_to_minimap(self.vehicle.get_location(), scale, offset)
        pygame.draw.circle(minimap_surface, (255, 255, 0), vehicle_pos, 5)

        # Scale and display the minimap
        scaled_minimap = pygame.transform.scale(minimap_surface, (160, 160))
        display.blit(scaled_minimap, (1080, 20))

    def calculate_minimap_scale_and_offset(self, vehicle, waypoints, minimap_size):
        # Calculate minimap scale and offset based on vehicle and waypoints
        all_points = [vehicle.get_location()] + [waypoint.transform.location for waypoint in waypoints]
        min_x = min(point.x for point in all_points)
        max_x = max(point.x for point in all_points)
        min_y = min(point.y for point in all_points)
        max_y = max(point.y for point in all_points)

        world_width = max_x - min_x
        world_height = max_y - min_y

        scale_x = minimap_size[0] / world_width if world_width > 0 else 1
        scale_y = minimap_size[1] / world_height if world_height > 0 else 1
        scale = min(scale_x, scale_y)

        vehicle_pos = vehicle.get_location()
        offset_x = (minimap_size[0] / 2) - (vehicle_pos.x - min_x) * scale
        offset_y = (minimap_size[1] / 2) - (vehicle_pos.y - min_y) * scale
        offset = (offset_x, offset_y)

        return scale, offset

    @staticmethod
    def world_to_minimap(location, scale, offset):
        # Convert world coordinates to minimap coordinates
        minimap_x = int(location.x * scale + offset[0])
        minimap_y = int(location.y * scale + offset[1])
        return (minimap_x, minimap_y)

if __name__ == "__main__":
    # Create and run the CarlaSimulation instance
    simulation = CarlaSimulation()
    simulation.setup()
    simulation.run()
