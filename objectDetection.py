# Import necessary libraries
import cv2
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO("yolov8.pt")

# Load an image
image = cv2.imread('test.jpg')

# Run the prediction
results = model.predict(image)

# Display the image and bounding boxes
for result in results:
    x1, y1, x2, y2 = result['box']
    label = result['label']
    cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
    cv2.putText(image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,0,0), 2)

cv2.imshow('Image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
