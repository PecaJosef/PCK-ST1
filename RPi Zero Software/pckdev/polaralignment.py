import camera
from star_processing import getAlignmentError
import math

def alignmentError(uart):
    star_image = captureImage(5,10,True) #10s exposure, 10 gain, flip the image
    NCPErrorX, NCPErrorY, polarisFound, PAsuccesful = getAlignmentError(star_image)
    
    pae_msg = f"#PAE:{int(PAsuccesful)}:{int(polarisFound)}:{NCPErrorX:.5f}:{NCPErrorY:.5f}"
    uart.send(pae_msg)
    return
