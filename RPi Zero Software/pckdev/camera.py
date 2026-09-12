from picamera2 import Picamera2
import cv2
import time
import numpy as np

PAcam = Picamera2()

def cameraConnected():
    cam_info = Picamera2.global_camera_info()
    if len(cam_info) == 0:
        return False
    else:
        return True

# Initialize camera
def initCamera():
    # 2x2 Binning config
    #config = PAcam.create_still_configuration(main={"size": (2028, 1520)})
    # Regular config
    #config = PAcam.create_still_configuration()

    config = PAcam.create_still_configuration(
        main={"size": (4056, 3040)},
        raw={"size": (2028, 1520), "format": "SRGGB12"}
    )
    
    PAcam.configure(config)
    PAcam.start()

def captureImage(exposure, gain, flip, raw = False): #Exposure [s], Gain [-], Flip [True/False], raw [True/False]
    PAcam.stop()
    #Set camera exposure and gain
    PAcam.set_controls({
    "AeEnable": False,
    "ExposureTime": int(exposure*1000000),   # in microseconds
    "AnalogueGain": gain
    })
    time.sleep(2)

    PAcam.start()
    
    if raw == True:
            # Fetch the raw 12-bit buffer
            raw_bytes = PAcam.capture_array("raw")
            raw_16bit = raw_bytes.view(np.uint16)
            # Demosaic raw Bayer pattern (RGGB) into BGR 16-bit for OpenCV
            image_out = cv2.cvtColor(raw_16bit, cv2.COLOR_BayerRG2BGR)
    else:
            # Fetch the standard ISP-processed RGB image
            image_rgb = PAcam.capture_array("main")
            image_out = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

    # Rotate image by 180 degrees if applicable
    if flip == True:
        image_rotated = cv2.rotate(image_bgr, cv2.ROTATE_180)
        return image_rotated
    else:
        return image_bgr
        
    

