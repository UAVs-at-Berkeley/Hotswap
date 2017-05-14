"""Camera Calibration File - Run this once to calculate the focal length of your camera on a red contour """

# import the necessary packages
import numpy as np
import cv2

#runs morphological operations on a binary image
def morphOps(mask):
    kernel = np.ones((5, 5),np.uint8)
    output = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel)
    return output

#returns the bounding rectangle around the largest red contour in the image
def find_marker(frame):
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lower = np.array([158, 94, 112], dtype = "uint8")
	upper = np.array([179, 255, 255], dtype = "uint8")
	mask = cv2.inRange(hsv, lower, upper)
	mask = morphOps(mask)
	contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if contours:
		max_cnt = max(contours, key=cv2.contourArea)
		return cv2.minAreaRect(max_cnt)

def findfocal(pixWidth):
    #compute the focal length given the width of the object in pixels
	return KNOWN_DISTANCE * pixWidth / KNOWN_WIDTH

KNOWN_WIDTH = 3.5 #initialize the object width (currently in inches but can change to any units)
KNOWN_DISTANCE = 12.0 #initialize the distance of the camera from the object (set the this before taking the picture (at that distance))

cap = cv2.VideoCapture(0) #initialize camera capture object

while(1):
	ret, frame = cap.read() #read a frame from the camera
	marker = find_marker(frame)
	if marker:
		# draw a bounding box around the image
		box = np.int0(cv2.cv.BoxPoints(marker))
		cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)

		cv2.imshow("frame", frame) #display the frame

        #press q when the red bounding box is exactly on the contour to take a picture and save it
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.imwrite("calibration.jpg", frame)
            break

image = cv2.imread("calibration.jpg") #read the image from the file and save it
marker = find_marker(image) #get the minAreaRect of the largest red blob in the image
print findfocal(marker[1][0]) #get the focal length and print to screen
