import cv2
import numpy as np

video = cv2.VideoCapture("maze_view.mp4")
        
          
vertices = np.array([[0,480],[0,100],[640,100],[640,480],
                         ], np.int32)  


def average_nearby_lines(lines, threshold_angle, threshold_dist):
    grouped_lines = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]  # Extract line endpoints
            line_slope=(y2-y1)/(x2-x1) 
            line_center=np.array([(x1+x2)/2,(y1+y2)/2])
            # print(line_slope)
            # Check if the line can be grouped with any existing group
            line_grouped = False
            for group in grouped_lines:
                
                #find the average of centre of lines in the group
                group_line = np.mean(group, axis=0, dtype=np.int32)
                x1, y1, x2, y2 = group_line
                group_center=np.array([(x1+x2)/2,(y1+y2)/2])
                group_slope=((y2-y1)/(x2-x1))
                
                # Calculate distance between centers
                # print("G",group_slope)
                if (abs(line_slope-group_slope) < threshold_angle)&(np.linalg.norm((group_center-line_center),ord=1)<threshold_dist):
                    group.append(line[0])  # Add line to the existing group
                    line_grouped = True
                    break

            # If the line does not belong to any existing group, create a new group
            if not line_grouped:
                grouped_lines.append([line[0]])

    averaged_lines = []
    for group in grouped_lines:
        group = np.array(group)
        averaged_line = np.mean(group, axis=0, dtype=np.int32)  # Calculate average line segment
        averaged_lines.append([averaged_line.tolist()])

    return averaged_lines
def roi(img, vertices):
    #blank mask:
    mask = np.zeros_like(img)
    # fill the mask
    cv2.fillPoly(mask, vertices, 255)
    # now only show the area that is the mask
    masked = cv2.bitwise_and(img, mask)
    return masked
threshold_theta = 0.5
threshold_delta = 400
# Adjust this threshold to control line grouping
while True:
    ret, frame = video.read()
   
   
    if not ret:
        video = cv2.VideoCapture("maze_view.mp4")
        continue
    frame = cv2.resize(frame, (640, 480))
 

    edges = cv2.Canny(frame, 100, 150)
    masked =  roi(edges, [vertices])           
    lines = cv2.HoughLinesP( masked, 2, np.pi/180, 100, minLineLength=200 , maxLineGap=10)
    lines = average_nearby_lines(lines, threshold_theta, threshold_delta)
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 30)
    # Average nearby lines

   
    cv2.imshow("edges", masked)
    cv2.imshow("frame", frame)
    key = cv2.waitKey(2)
    if key == 27:
        break
video.release()
cv2.destroyAllWindows()