import cv2
import processing

video = cv2.VideoCapture("beacon_test.mp4")
#Actual computer vision stuff
while True:
    
    ret, frame = video.read()
    
   
    if not ret:
        video = cv2.VideoCapture("beacon_test.mp4")
        continue
    
    processing.analyse_frame(frame)
    
    
    #save frame
    # if count%15==0:
    #     cv2.imwrite("frame%d.jpg" % count, frame)
    
    
    key = cv2.waitKey(250)
    if key == 27:
        break
    #count += 1
video.release()