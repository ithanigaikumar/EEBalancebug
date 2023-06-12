import cv2
import numpy as np
import cv_utils as cvu
#video = cv2.VideoCapture("maze_view.mp4")
import dfs_traversal
  
kernel = np.ones((5,5),np.uint8)    
   
vertices= np.array([[0,480],[0,270],[640,270],[640,480],
                         ], np.int32)  
forward_mask= np.array([[200,350],[200,270],[440,270],[440,350],
                         ], np.int32)  
left_mask = np.array([[0,480],[0,0],[320,0],[320,480],
                         ], np.int32)  
right_mask = np.array([[320,480],[320,0],[640,0],[640,480],
                         ], np.int32)  
#count for exporting frames
count=0

# Specify size on horizontal axis


# utility functions because cant have them in seperate files for pyscript








#Movement and graph building logic
def descision(walls):
    if walls==[1,1,0]:
       return "Right"
    elif walls==[0,1,1]:
       return  "Left"
    elif walls==[1,0,1]:
     return "Forward"
    elif walls==[1,1,1]:
     return "Backtrack"
    else:
        return "Undefined"
def filterHSV(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      
    # Threshold of blue in HSV space
    lower_blue = np.array([20, 50, 140])
    upper_blue = np.array([180, 255, 255])
  
    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
      
    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions
    result = cv2.bitwise_and(frame, frame, mask = mask)
    return result
    
def calculate_height(y1, y2, pixel_to_cm):
# Assuming the height of the camera is known, convert pixel height to centimeters
    height_cm = (y2 - y1) * pixel_to_cm
    return height_cm
  
def draw_bounding_boxes(frame, pixel_to_cm):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (1, 1), 0)
    _, thresh = cv2.threshold(blur, 127, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Approximate the contour as a circle
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)

        # Calculate bounding box coordinates
        x1, y1 = int(x - radius), int(y - radius)
        x2, y2 = int(x + radius), int(y + radius)

        # Draw the bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Calculate the height in centimeters
        height_cm = calculate_height(y1, y2, pixel_to_cm)
        distance_cm = (8206.58624* 4.1) / height_cm
        # Display the height
        cv2.putText(frame, f"distance: {distance_cm:.2f} cm", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    return frame





def analyseFrame(frame):
    pixel_to_cm = 1 
    filtered_frame=filterHSV(frame)
    filtered_frame=draw_bounding_boxes(filtered_frame, pixel_to_cm)
    #frame = cv2.resize(frame, (640, 480))
    edges = cv2.Canny(frame, 150, 255)
    edges=cvu.roi(edges,[vertices])
    #edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    masks=[left_mask,forward_mask,right_mask]
    walls=[0,0,0]
    #frame=draw_grid(frame,(2,2))
    #cv2.circle(frame, (320,240), 6, (0,255,0), 1)
    
    line_buffer=[None,None,None]
    for i in range(0,len(masks)):
        mask=masks[i]
        lines = cv2.HoughLinesP(cvu.roi(edges,[mask]),1, np.pi/180, threshold=150, minLineLength=50 , maxLineGap=5)
        bundler = cvu.HoughBundler(min_distance=10,min_angle=2)
        lines = bundler.process_lines(lines,(i!=1))
       
        if lines !=[]:
            walls[i]=1
            lines=cvu.closest_to_center(lines)
            line_buffer[i]=lines[0]
        else:
            walls[i]=0
            if(i==0):
                lines=[[0,480,0,0]]
                line_buffer[i]=[0,480,0,0]
            elif(i==2):
                lines=[[640,480,640,0]]
                line_buffer[i]=[640,480,640,0]
        cvu.draw_lines(lines,frame)
    offset=(cvu.GetVanishingPoint([line_buffer[0],line_buffer[2]]))
    #print(offset)
    #cv2.circle(frame, tuple(offset), 6, (0,0,255), 5)
    cvu.__draw_label(frame, 'offset = %d' % (offset[0]-320), (20,20), (255,255,255))
    cvu.__draw_label(frame, 'walls = [%d , %d, %d]' % tuple(walls), (20,40), (255,255,255))
    cvu.__draw_label(frame, 'descision = %s' % descision(walls), (20,60), (255,255,255))
    cv2.imshow("edges", edges)
    cv2.imshow('Bounding Boxes', filtered_frame)
    cv2.imshow("frame", frame)
    cv2.waitKey(75)
    return walls
    
#Actual computer vision stuff
# while True:
    
#     ret, frame = video.read()
    
   
#     if not ret:
#         video = cv2.VideoCapture("maze_view.mp4")
#         continue
    
#     analyseFrame(frame)
    
    
#     #save frame
#     # if count%15==0:
#     #     cv2.imwrite("frame%d.jpg" % count, frame)
    
    
#     key = cv2.waitKey(25)
#     if key == 27:
#         break
#     count += 1
# video.release()
cv2.destroyAllWindows()