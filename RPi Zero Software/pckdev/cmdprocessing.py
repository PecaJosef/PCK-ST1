from camera import captureImage
from star_processing import getAlignmentError, getCenterOfRotation
import math
import subprocess
import time
import gc
import numpy as np

import cv2

image_zero = None
image_angled = None
center_offset = None

def alignmentError(uart):
    global center_offset
    #star_image = cv2.imread("star_img/polaris4_1.jpg")

    star_image = captureImage(10, 8.0, False) #10s exposure, 8 gain, flip the image

    cv2.imwrite('stars.jpg', star_image)
    uart.send("$PA:CAPTURED")

    if (center_offset is None):
        center_offset = np.array([0, 0])

    NCPErrorX, NCPErrorY, polarisFound, ncpFound, raAngle = getAlignmentError(star_image, center_offset, True)
    pae_msg = f"$PA:DEV:{int(ncpFound)}:{int(polarisFound)}:{-NCPErrorX:.5f}:{-NCPErrorY:.5f}:{raAngle:.5f}"
    uart.send(pae_msg)
    return

def centerOfRotation(uart):
    global image_zero, image_angled, center_offset

    star_image = captureImage(10, 8.0, False) #10s exposure, 8 gain, flip the image
    uart.send("$PA:CAPTURED")

    if (image_zero is None):
        image_zero = star_image
        cv2.imwrite('cor1.jpg', image_zero)
    else:
        image_angled = star_image
        cv2.imwrite('cor2.jpg', image_angled)
    
    if(image_zero is not None and image_angled is not None):
        center_offset = getCenterOfRotation(image_zero, image_angled)
        if center_offset is not None:
            uart.send("$PA:COR:DONE")
            time.sleep(0.25)
            uart.send("#CoR calculated")
        else:
            uart.send("$PA:COR:FAIL")
            time.sleep(0.25)
            uart.send("#CoR failed!")
        
        image_zero = None
        image_angled = None
        gc.collect()

def imgCapture(uart, exposure, img_name):
    image = captureImage(exposure, 8.0, False)
    cv2.imwrite(img_name, image)
    uart.send("#IMAGE CAPTURED")

def rpiShutdown(uart):
    print("[SYSTEM] Shutting down Raspberry Pi...")
    uart.send("$PWROFF")
    time.sleep(0.25)
    uart.send("#Powering off OK")
    time.sleep(0.25)
    subprocess.run(["sudo", "shutdown", "-h", "now"],check=False)