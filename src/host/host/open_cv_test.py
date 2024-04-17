import cv2


def main():
    print("hello!")


#   # Create a named window for the preview
#   cv2.namedWindow("preview")

#   # Initialize the webcam (camera index 0)
#   vc = cv2.VideoCapture(0)

#   # Check if the webcam opened successfully
#   if vc.isOpened():
#       rval, frame = vc.read()
#   else:
#       rval = False

#   # Continuously display frames from the webcam
#   while rval:
#       cv2.imshow("preview", frame)
#       rval, frame = vc.read()

#       # Exit the loop when the user presses the ESC key
#       key = cv2.waitKey(20)
#       if key == 27:
#           break

#   # Clean up: destroy the window and release the webcam
#   cv2.destroyWindow("preview")
#   vc.release()
