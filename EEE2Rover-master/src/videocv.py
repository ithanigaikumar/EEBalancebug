import cv2
import numpy as np

video = cv2.VideoCapture("maze_view.mp4")
        
          
vertices = np.array([[0,700],[0,200],[1440,200],[1440,700],
                         ], np.int32)  

def roi(img, vertices):
    #blank mask:
    mask = np.zeros_like(img)
    # fill the mask
    cv2.fillPoly(mask, vertices, 255)
    # now only show the area that is the mask
    masked = cv2.bitwise_and(img, mask)
    return masked
while True:
    ret, frame = video.read()
   
   
    if not ret:
        video = cv2.VideoCapture("maze_view.mp4")
        continue
    frame = cv2.resize(frame, (1280, 720))
    mask =  roi(frame, [vertices])
    edges = cv2.Canny(mask, 75, 150)
 
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=100)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
    
    cv2.imshow("edges", edges)
    cv2.imshow("frame", frame)
    key = cv2.waitKey(25)
    if key == 27:
        break
video.release()
cv2.destroyAllWindows()