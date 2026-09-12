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
    #config = PAcam.create_still_configuration(main={"size": (4056, 3040)})

    config = PAcam.create_still_configuration(
        main={"size": (4056, 3040)},
        raw={"size": (4056, 3040), "format": "SRGGB12"}
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

            image_cropped = raw_16bit[:3040, :4056]
            blocks = image_cropped.reshape(1520, 2, 2028, 2)
            image_binned = (blocks.astype(np.uint32).sum(axis=(1, 3)) // 4).astype(np.uint16)

            image_out = image_binned * 16

    else:
            # Fetch the standard ISP-processed RGB image
            image_rgb = PAcam.capture_array("main")
            image_out = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

    # Rotate image by 180 degrees if applicable
    if flip == True:
        image_out = cv2.rotate(image_out, cv2.ROTATE_180)
        
    return image_out
        
    

