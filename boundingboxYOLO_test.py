import carla
import random
import queue
import numpy as np
import pygame

import cv2
from PIL import Image
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO("yolov8n.pt")

# Frame counter
frame_counter = 0

client = carla.Client('localhost', 2000)
world  = client.get_world()
bp_lib = world.get_blueprint_library()

# spawn vehicle
# Get the map spawn points
spawn_points = world.get_map().get_spawn_points()
vehicle_bp =bp_lib.find('vehicle.lincoln.mkz_2020')
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# spawn camera
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_init_trans = carla.Transform(carla.Location(z=2))
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
vehicle.set_autopilot(True)

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)



# Create a queue to store and retrieve the sensor data
image_queue = queue.Queue()
camera.listen(image_queue.put)

def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

def get_image_point(loc, K, w2c):
        # Calculate 2D projection of 3D coordinate

        # Format the input coordinate (loc is a carla.Position object)
        point = np.array([loc.x, loc.y, loc.z, 1])
        # transform to camera coordinates
        point_camera = np.dot(w2c, point)

        # New we must change from UE4's coordinate system to an "standard"
        # (x, y ,z) -> (y, -z, x)
        # and we remove the fourth componebonent also
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # now project 3D->2D using the camera matrix
        point_img = np.dot(K, point_camera)
        # normalize
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]

# Get the world to camera matrix
world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

# Get the attributes from the camera
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()
fov = camera_bp.get_attribute("fov").as_float()

# Calculate the camera projection matrix to project from 3D -> 2D
K = build_projection_matrix(image_w, image_h, fov)

# Retrieve all bounding boxes for traffic lights within the level
# We filter for traffic lights and traffic signs
bounding_box_set = world.get_level_bbs(carla.CityObjectLabel.TrafficLight)
bounding_box_set.extend(world.get_level_bbs(carla.CityObjectLabel.TrafficSigns))

# Filter the list to extract bounding boxes within a 50m radius
# nearby_bboxes = []
# for bbox in bounding_box_set:
#     if bbox.location.distance(vehicle.get_transform().location) < 50:
#         nearby_bboxes

edges = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

# Initialize Pygame and create a window
pygame.init()
display = pygame.display.set_mode((image_w, image_h))

while True:
    world.tick()
    image = image_queue.get()
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

    #extra setup for pygame start
    img = img[:, :, :3]
    img = img[:, :, ::-1]

    # Downsample the image
    #img_yolo = cv2.resize(img, (image.width // 2, image.height // 2))
    img_yolo = img

    img = pygame.surfarray.make_surface(img.swapaxes(0, 1))
    display.blit(img, (0, 0))
    pygame.display.flip()
    #end

    #YOLO...

    # Run the prediction every 5 frames
    if frame_counter % 60 == 0:
        results = model.predict(img_yolo)

        # View results
        for r in results:
            #print(r)
            #print(r.boxes)  # print the Boxes object containing the detection bounding boxes
            yolo_boxes = r.boxes.xywh
            for box in yolo_boxes:
                x1, y1, w, h = box  # Extract coordinates from YOLO output
                pygame.draw.rect(display, (255,0,0), (x1, y1, w, h), 2)  # Draw bounding box
        
    frame_counter += 1
    #End

    # Draw bounding boxes using the YOLO detection output
    
        
    # # Get the camera matrix 
    # world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

    # for bb in bounding_box_set:

    #     # Filter for distance from ego vehicle
    #     if bb.location.distance(vehicle.get_transform().location) < 50:

    #         # Calculate the dot product between the forward vector
    #         # of the vehicle and the vector between the vehicle
    #         # and the bounding box. We threshold this dot product
    #         # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
    #         forward_vec = vehicle.get_transform().get_forward_vector()
    #         ray = bb.location - vehicle.get_transform().location

    #         if forward_vec.dot(ray) > 1:
    #             # Cycle through the vertices
    #             verts = [v for v in bb.get_world_vertices(carla.Transform())]
    #             for edge in edges:
    #                 # Join the vertices into edges
    #                 p1 = get_image_point(verts[edge[0]], K, world_2_camera)
    #                 p2 = get_image_point(verts[edge[1]],  K, world_2_camera)
    #                 # Draw the edges into the camera output                    
    #                 pygame.draw.line(display, (255, 0, 0), (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), 1)
    
    pygame.display.update()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
