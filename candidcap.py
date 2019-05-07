import cv2
import numpy as np

# cap = cv2.VideoCapture(2)

def findcontours(frame):

    # Take each frame
    # _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    lower_blue = np.array([0, 100, 100])
    upper_blue = np.array([8, 255, 255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx, cy = 0, 0
    if contours :
	j = contours[0]
	maxm = cv2.contourArea(contours[0])
	for i in contours :
	    area = cv2.contourArea(i)
	    if area > maxm :
	    	maxm = area
	    	j = i

    # Bitwise-AND mask and original image
	M = cv2.moments(j)
	if M['m00'] != 0 :
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

    return (cx, cy)


#     cv2.imshow('frame',frame)
#     k = cv2.waitKey(5) & 0xFF
#     if k == 27:
#         break

# cap.release()
# cv2.destroyAllWindows()
