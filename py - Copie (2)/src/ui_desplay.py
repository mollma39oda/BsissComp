import cv2 as cv
from config import SHOW_MASKS, SHOW_MASKS_BITWISE, SHOW_FRAME

def CV_Imshow_Display(frame, masked, bitwised):
    if SHOW_FRAME:
        if frame is not None:
            cv.imshow("processed image", frame)
        else:
            print("message from ui_display: frame is none")
    
    if SHOW_MASKS:
        if masked is not None:
            cv.imshow("processed mask", masked)
        else:
            print("message from ui_display: masked image is none")
    
    if SHOW_MASKS_BITWISE:
        if bitwised is not None:
            cv.imshow("processed bitwise", bitwised)
        else:
            print("message from ui_display: bitwised is none")
            
def App_Display(frame, masked, bitwised):
    print("This app not developed yet")
    CV_Imshow_Display(frame, masked, bitwised)
       