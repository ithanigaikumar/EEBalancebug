import cv2
import processing

video = cv2.VideoCapture("beacon_sweep.mp4")
#Actual computer vision stuff
while True:
    
    ret, frame = video.read()
    if not ret:
        video = cv2.VideoCapture("beacon_test.mp4")
        continue

    filtered_frame=processing.filterHSV(frame)
    filtered_frame=processing.draw_bounding_boxes(filtered_frame, 1 )
    cv2.imshow("beacon", filtered_frame)
    #save frame
    # if count%15==0:
    #     cv2.imwrite("frame%d.jpg" % count, frame)
    key = cv2.waitKey(250)
    
   
    if key == 27:
        break
    #count += 1
video.release()
cv2.destroyAllWindows()