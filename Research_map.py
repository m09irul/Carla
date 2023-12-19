import carla
import math
import random

# Connect to the CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(20.0)

# Get the world
world = client.get_world()

# Get the list of vehicles
vehicles = world.get_actors().filter('vehicle.*')

# If there are vehicles, get the first one
if vehicles:
    vehicle = vehicles[0]
else:
    # No vehicles found, spawn one
    print('No vehicles found, spawning one...')
    
    # Get the blueprint library from the CARLA world
    blueprint_library = world.get_blueprint_library()
    # Get a random blueprint for a vehicle
    blueprint = random.choice(blueprint_library.filter('vehicle.*'))
    
    # Choose a random location to spawn the vehicle
    transform = random.choice(world.get_map().get_spawn_points())
    
    # Spawn the vehicle
    vehicle = world.spawn_actor(blueprint, transform)


# Get the blueprint library from the CARLA world
blueprint_library = world.get_blueprint_library()

# Get the GNSS sensor blueprint
gnss_bp = blueprint_library.find('sensor.other.gnss')

# Set the attributes of the GNSS sensor as needed
# For example, you can set the frequency of the sensor
gnss_bp.set_attribute('sensor_tick', '0.1')

# Spawn the GNSS sensor and attach it to your vehicle
transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.0))
gnss_sensor = world.spawn_actor(gnss_bp, transform, attach_to=vehicle)

# Define a function to get the next waypoint
def get_next_waypoint(gnss_measurement):
    # Convert the GNSS measurement to a CARLA location
    location = carla.Location(x=gnss_measurement.latitude, y=gnss_measurement.longitude, z=gnss_measurement.altitude)

    # Get the waypoint corresponding to the current location
    current_waypoint = map.get_waypoint(location)

    # Get the next waypoints
    next_waypoints = current_waypoint.next(1.0)

    # If there are no next waypoints, return None
    if not next_waypoints:
        return None

    # Choose the next waypoint that is closest to the destination
    next_waypoint = min(next_waypoints, key=lambda waypoint: waypoint.transform.location.distance(destination))

    return next_waypoint

# Define a callback function to process the GNSS data
def process_gnss_data(gnss_measurement):
    print(f'GNSS Measurement: {gnss_measurement}')

    # Get the next waypoint
    next_waypoint = get_next_waypoint(gnss_measurement)

    # Drive to the next waypoint
    #drive_to_waypoint(next_waypoint)  # Replace with your function to drive to the waypoint

# Register the callback function to be called whenever a new measurement is received
#gnss_sensor.listen(process_gnss_data)

# Get the spectator from the world
spectator = world.get_spectator()

# Define a function to update the spectator's transform
def update_spectator_transform():
    # Get the vehicle's transform
    vehicle_transform = vehicle.get_transform()

    # Calculate the rotation from the vehicle's velocity
    velocity = vehicle.get_velocity()
    yaw = -1.0 * math.degrees(math.atan2(velocity.y, velocity.x))
    rotation = carla.Rotation(yaw=yaw)

    # Set the spectator's transform
    spectator.set_transform(carla.Transform(vehicle_transform.location + carla.Location(z=50), rotation))

# Update the spectator's transform in a loop
while True:
    update_spectator_transform()
