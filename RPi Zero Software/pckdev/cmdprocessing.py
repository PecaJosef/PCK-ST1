from camera import captureImage
from star_processing import getAlignmentError
import math
import subprocess
import time

import cv2

def alignmentError(uart):
    #star_image = cv2.imread("star_img/polaris4_1.jpg")

    star_image = captureImage(10, 8.0, True) #10s exposure, 8 gain, flip the image

    cv2.imwrite('stars.jpg', star_image)

    NCPErrorX, NCPErrorY, polarisFound, ncpFound = getAlignmentError(star_image)
    pae_msg = f"$PA:DEV:{int(ncpFound)}:{int(polarisFound)}:{-NCPErrorX:.5f}:{-NCPErrorY:.5f}"
    uart.send(pae_msg)
    return

def rpiShutdown(uart):
    print("[SYSTEM] Shutting down Raspberry Pi...")
    uart.send("#SHUTDOWN:OK")
    time.sleep(0.25)
    subprocess.run(["sudo", "shutdown", "-h", "now"],check=False)