from picamera2 import Picamera2
import cv2
import time

PAcam = Picamera2()

def cameraConnected():
    cam_info = Picamera2.global_camera_info()
    if len(cam_info) == 0:
        return False
    else:
        return True

# Initialize camera
def initCamera():
    PAcam.configure(PAcam.create_still_configuration())
    PAcam.start()

def captureImage(exposure, gain, flip): #Exposure [s], Gain [-], Flip [True/False]
    PAcam.stop()
    #Set camera exposure and gain
    PAcam.set_controls({
    "AeEnable": False,
    "ExposureTime": int(exposure*1000000),   # in microseconds
    "AnalogueGain": gain
    })
    time.sleep(2)

    PAcam.start()
    
    # Capture image as RGB
    image_rgb = PAcam.capture_array()

    # Convert to BGR for OpenCV - not mandatory, later turned to greyscale anyway
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    #image_gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)

    # Rotate image by 180 degrees
    if flip == True:
        image_rotated = cv2.rotate(image_bgr, cv2.ROTATE_180)
        return image_rotated
    else:
        return image_bgr
        
    

