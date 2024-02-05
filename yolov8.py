# Import necessary libraries
import cv2
from PIL import Image
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO("yolov8n.pt")

# Load an image
image = cv2.imread('road.jpg')
#image = 'road.jpg'

# Run the prediction
results = model.predict(image)

# View results
for r in results:
    print(r.boxes)  # print the Boxes object containing the detection bounding boxes

# Show the results
for r in results:
    im_array = r.plot()  # plot a BGR numpy array of predictions
    im = Image.fromarray(im_array[..., ::-1])  # RGB PIL image
    im.show()  # show image
    im.save('results.jpg')  # save image