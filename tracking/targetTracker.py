import cv2
import numpy as np
import math

import base64
import time
import urllib
import urllib.request

APERTURE_ANGLE = 30.0 * math.pi / 180.0 #radians
LONGSIDE_ACTUAL = 4.0 #inches rn
SCALE = 8.0 #for scaling arrow vector
OFF_TARGET_TOLERANCE = 10 #pixel difference from center of target to consider on/off target; also used as radius
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
BOUNDING_RATIO = 1.675
cent = (FRAME_WIDTH//2, FRAME_HEIGHT//2)

surf = cv2.xfeatures2d.SURF_create(300)


class ipCamera(object):

    def __init__(self, url, user=None, password=None):
        self.url = url
        # auth_encoded = base64.encodestring('%s:%s' % (user, password))[:-1]

        self.req = urllib.request.Request(self.url)
        # self.req.add_header('Authorization', 'Basic') #%s' % auth_encoded)

    def get_frame(self):
        print("GETTING FRAME")
        response = urllib.request.urlopen(self.req)
        print("GOT FRAME")
        img_array = np.asarray(bytearray(response.read()), dtype=np.uint8)
        frame = cv2.imdecode(img_array, 1)
        return frame



def withinRatioTol(rect):
    ((x,y), (width, height), angle) = rect
    return False if width == 0 or height == 0 else abs(width / height - BOUNDING_RATIO) < 0.2 or abs(height / width - BOUNDING_RATIO) < 0.2

def rectDistanceAngle(rect):
    ((x,y), (width, height), angle) = rect
    longside_pix = max(width, height)
    return (int(x-cent[0]), int(y-cent[1])), FRAME_WIDTH * LONGSIDE_ACTUAL / longside_pix / (2 * math.tan(APERTURE_ANGLE));

def processSurf(original, logo, keypoint_obj, descriptors_obj):

    def distance(p1, p2):
        return pow ( pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2), 0.5 )

    m = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)

    keypoints_scene, descriptors_scene = surf.detectAndCompute(m, None)

    matcher = cv2.BFMatcher(cv2.NORM_L2, True)
    matches = matcher.knnMatch(descriptors_obj, descriptors_scene, k=1)

    max_dist, min_dist = 0,100

    # print(descriptors_obj)
    # print(descriptors_scene)
    # print(matches)

    for match in matches:
        if len(match) > 0:
            dist = match[0].distance
            if dist < min_dist: 
                min_dist = dist
            if dist > max_dist:
                max_dist = dist

    medium_matches = []
    scene_raw = []
    for match in matches:
        if len(match) > 0:
            query = match[0].queryIdx
            train = match[0].trainIdx
            if query < len(keypoint_obj) and train < len(keypoints_scene): #and match[0].distance < 0.98 * match[1].distance:
                medium_matches.append(match[0])
                scene_raw.append( keypoints_scene[train].pt )
    
    #for filtering based on distance away from mean point
    mean_x = int(sum([x for x,y in scene_raw])/len(scene_raw))
    mean_y = int(sum([y for x,y in scene_raw])/len(scene_raw))
    # print(distance)
    distances = [ distance(pt, (mean_x, mean_y)) for pt in scene_raw]
    distances = sorted(distances)
    maxDist = distances[-1]
    medianDist = distances[len(distances)//2]
    threshDist = medianDist + (maxDist-medianDist)/2

    obj = []
    scene = []

    good_matches = []

    for match in medium_matches:
        pt_obj = keypoint_obj[ match.queryIdx ].pt
        pt_scene = keypoints_scene[ match.trainIdx ].pt

        if match.distance < 0.80 * max_dist and distance((mean_x, mean_y), pt_scene) < threshDist:
            good_matches.append(match)
            obj.append(pt_obj)
            scene.append(pt_scene)

    obj = np.array(obj)
    scene = np.array(scene)

    img_matches = None

    if len(good_matches) > 4:
        #finally done filtering
        # print(good_matches)
        img_matches = cv2.drawMatches( logo, keypoint_obj, original, keypoints_scene, matches1to2=good_matches, outImg=None, flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

        cv2.circle(img_matches, (mean_x + logo.shape[1], mean_y), 10, (0,0,255), 2)

        #proceed if we have good matches
        H, mask = cv2.findHomography(obj, scene, method=cv2.RANSAC)
        logo_corners = np.array([[0,0], [logo.shape[1], 0], [logo.shape[1], logo.shape[0]], [0, logo.shape[0]]], dtype='float32')
        logo_corners = np.array([logo_corners])
        if H is not None and H.shape[0] > 1:
            #if there is a homography mat to use
            # print(H)
            scene_corners = cv2.perspectiveTransform(logo_corners, H)
            rect = cv2.minAreaRect(scene_corners)
            if withinRatioTol(rect):
                rect_points = cv2.boxPoints(rect)

                correctionVec, dist = rectDistanceAngle(rect)
                mag = distance((0,0), correctionVec)
                norm = (SCALE / mag * correctionVec[0],  SCALE / mag * correctionVec[1])
                #  [ 1/rt(2) -1/rt(2)  ;  1/rt(2) 1/rt(2) ] * scaled_vec
                shift1 = (norm[0]/math.sqrt(2) - norm[1]/math.sqrt(2),
                                    norm[0]/math.sqrt(2) + norm[1]/math.sqrt(2))
                shift2 = (norm[0]/math.sqrt(2) + norm[1]/math.sqrt(2),
                                    - norm[0]/math.sqrt(2) + norm[1]/math.sqrt(2))
                
                if mag < OFF_TARGET_TOLERANCE:
                    cv2.circle(original, cent, OFF_TARGET_TOLERANCE, (0,255,0), 3)
                else:
                    cv2.line( original, (cent[0] + correctionVec[0],cent[1] + correctionVec[1]) , cent, (0,0,255), 2);
                    cv2.line( original, cent, (cent[0] + int(shift1[0]), cent[1] + int(shift1[1])), (0,0,255),2);
                    cv2.line( original, cent, (cent[0] + int(shift2[0]), cent[1] + int(shift2[1])), (0,0,255),2);
                cv2.putText(original, "Distance: " + "{0:.2f}".format(dist), (40,40), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,255,0), 2)

                for j in range(4):
                    cv2.line( original, tuple(rect_points[j]), tuple(rect_points[(j+1)%4]), (0,0,255), 2)
                    cv2.line( img_matches, (int(rect_points[j][0] + logo.shape[1]), int(rect_points[j][1])), 
                        (int(rect_points[(j+1)%4][0] + logo.shape[1]), int(rect_points[(j+1)%4][1])), (255, 0, 255), 4);
    # if img_matches is None :
    #     return original
    # return img_matches
    return original



print("######### PYTHON DETECTION STARTING ########\n")

logo = cv2.imread('cal_logo_uavs.png', cv2.IMREAD_GRAYSCALE)
keypoints_obj, descriptors_obj = surf.detectAndCompute(logo, None)

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture("http://192.168.0.101:8080/video?.mjpg")
# cap = ipCamera("http://192.168.0.101:8080/video?.mjpg")

cv2.namedWindow('Image',cv2.WINDOW_AUTOSIZE)

while(True):
    # Capture frame-by-frame
    ret, original = cap.read() #cap.get_frame()
    if original is not None and original.shape[0] > 1:
        original = cv2.resize(original, (int(original.shape[1]/1.5), int(original.shape[0]/1.5)))

        # Our operations on the frame come here
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        original = processSurf(original, logo, keypoints_obj, descriptors_obj)

        # Display the resulting frame
        cv2.imshow('Image',original)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()




