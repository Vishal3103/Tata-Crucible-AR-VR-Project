import numpy as np
import cv2
#from matplotlib import pyplot as plt
# from webcam import *
# webcam = Webcam()
# webcam.start()

MIN_MATCH_COUNT = 10

img1 = cv2.imread('box.png',0)          # queryImage
# img2 = cv2.imread('box_in_scene.png',0) # trainImage

# Initiate SIFT detector
sift = cv2.xfeatures2d.SIFT_create()

# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(img1,None)
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv2.FlannBasedMatcher(index_params, search_params)
# kp2, des2 = sift.detectAndCompute(img2,None)
def get_vectors(image, points):
 
    with np.load('webcam_calibration_ouput.npz') as X:
        mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
   
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
 
    imgp = np.array(points, dtype="float32")
 
    objp = np.array([[0.,0.,0.],[1.,0.,0.],
                        [1.,1.,0.],[0.,1.,0.]], dtype="float32")  

    cv2.cornerSubPix(gray,imgp,(11,11),(-1,-1),criteria)
    _, rvecs, tvecs, _ = cv2.solvePnPRansac(objp, imgp, mtx, dist)
 
    return rvecs, tvecs

def detect(img2) :
	gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
	kp2, des2 = sift.detectAndCompute(gray, None)

	matches = flann.knnMatch(des1,des2,k=2)

	# store all the good matches as per Lowe's ratio test.
	good = []
	for m,n in matches:
	    if m.distance < 0.7*n.distance:
	        good.append(m)

	if len(good)>MIN_MATCH_COUNT:
	    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
	    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

	    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
	    matchesMask = mask.ravel().tolist()

	    h,w = img1.shape
	    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
	    dst = cv2.perspectiveTransform(pts,M)

	    rvecs, tvecs = get_vectors(img2, dst)

	    return (dst, rvecs, tvecs, img2, True)

	else:
	    # print("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))

	    return (0, 0, 0, img2, False)

	# draw_params = dict(matchColor = (0,255,0), # draw matches in green color
	#                    singlePointColor = None,
	#                    matchesMask = matchesMask, # draw only inliers
	#                    flags = 2)

	# img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
	# plt.imshow(img3, 'gray'),plt.show()

# while True :
# 	cv2.imshow('frame', detect(webcam.get_current_frame())[0])
# 	if cv2.waitKey(1) & 0xFF == ord('q') :
# 		break

# webcam.video_capture.release()
# cv2.destroyAllWindows()
