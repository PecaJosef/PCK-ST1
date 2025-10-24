from picamera2 import Picamera2
import cv2
import time

# Initialize camera
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration())
picam2.start()

# Allow warm-up
time.sleep(1)

# Set manual exposure and gain
picam2.set_controls({
    "AeEnable": False,
    "ExposureTime": 100000,   # in microseconds
    "AnalogueGain": 4.0
})

time.sleep(0.2)

# Capture image as RGB
image_rgb = picam2.capture_array()

# Convert to BGR for OpenCV
image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

# Rotate image by 180 degrees
image_rotated = cv2.rotate(image_bgr, cv2.ROTATE_180)

# Save image
cv2.imwrite("img.jpg", image_rotated)

print("Image captured and saved as img.jpg (rotated 180°).")