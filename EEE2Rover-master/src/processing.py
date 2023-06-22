import cv2
import numpy as np
import cv_utils as cvu
import perspective_utils
#import mapping
#video = cv2.VideoCapture("maze_view.mp4")

  
kernel = np.ones((5,5),np.uint8)    
   
vertices= np.array([[0,480],[0,100],[640,100],[640,480],
                         ], np.int32)  
null_mask= np.array([[0,0],[0,0],[0,0],[0,0],
                         ], np.int32)  
front_mask= np.array([[0,340],[0,100],[640,100],[640,340],
                         ], np.int32)  
left_mask = np.array([[0,480],[0,100],[320,100],[320,480],
                         ], np.int32)  
right_mask = np.array([[320,480],[320,100],[640,100],[640,480],
                         ], np.int32)  
#count for exporting frames
count=0



# utility functions because cant have them in seperate files for pyscript

#Movement and graph building logic
def state(p1,p2,p3,p4,p5,p6):
    if p1:
        return 1 #empty space
    if p2:
        return 2 #facing a wall
    if p3 and p5:
        return 3 # corridor
    if p3 and (not p5) and (not p6):
        return 4 #dangerous turn
    if p3 and (not p5) and p6:
        return 5 #facing edge
    if p4:
        return 6 #approaching a turn
    
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
        if radius>0:
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

def decode_state(state):
    if state==1:
        return "empty space"
    if state==2:
        return "facing a wall"
    if state==3:
        return "corridor"
    if state==4:
        return "dangerous turn"
    if state==5:
        return "facing edge"
    if state==6:
        return "approaching a turn"

def action(current_state,next_state):
    if current_state==0:#entry state
        return 0
    if current_state==1: #empty space
        if next_state==1:
            return 2
        elif next_state==2:
            return 2 # set back to 2
    if current_state==2: #facing a wall
        if next_state==6:
            return 3 
        elif next_state==2:
            return 1 # go forward
        elif next_state==3:
            return 3 
    if current_state==3: #corridor
        if next_state == 3:
            return 4
        elif next_state == 6:
            return 4
    if current_state==4:
        if next_state==4:
            return 3
        if next_state==6:
            return 3
    if current_state==5:
        if next_state==5:
            return 3
        if next_state==3:
            return 3
    if current_state==6:
        if next_state==2:
            return 4
        if next_state==4:
            return 4
        if next_state==6:
            return 4
        if next_state==5:
            return 4
    return 0
def points_from_action(action,line_buffer):
    linear_vel=0.5
    if action==0:
        return 320,320,0
    elif action==1:
        s1=0
        s2=1
        linear_vel=0
    elif action==2:
        s1=0
        s2=1
    elif action==3:
        s1=0
        s2=2
    elif action==4:
        s1=0
        s2=2
    x_m=cvu.find_mid_point([line_buffer[s1],line_buffer[s2]])[0]
    x_v=cvu.find_vanishing_point([line_buffer[s1],line_buffer[s2]])[0]
    return x_m,x_v,linear_vel
    
def analyse_frame(frame,current_state,x_z_position,y_rotation):
    frame = cv2.resize(frame, (640, 480))
   
    pixel_to_cm = 1 
    filtered_frame=filterHSV(frame)
    filtered_frame=draw_bounding_boxes(filtered_frame, pixel_to_cm)
    edges = cv2.Canny(frame, 150, 255)
    kernel = np.ones((3,3), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations = 1)
    edges=cvu.roi(edges,[vertices])
    #edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    masks=[left_mask,front_mask,right_mask]
    walls=[0,0,0]
    #frame=cvu.draw_grid(frame,(2,2))

    # frame=cv2.rectangle(frame,vertices[0],vertices[2],(0,255,0),1) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!debug
    # frame=cv2.rectangle(frame,front_mask[0],front_mask[2],(0,255,0),1)
    # frame=cv2.rectangle(frame,left_mask[0],left_mask[2],(0,255,0),1)
    # frame=cv2.rectangle(frame,right_mask[0],right_mask[2],(0,255,0),1)

    #cv2.circle(frame, (320,240), 6, (0,255,0), 1)
    # brush=perspective_utils.birdeye(frame)[0]
    
    # map=mapping.overlay_image(map,brush,(20*x_z_position[0]+40,9*100-20*x_z_position[1]-40),(40,40),y_rotation)
    debug_frame=filtered_frame.copy()
    #linesP = cv2.HoughLinesP(edges,1, np.pi/180, threshold=100, minLineLength=10 , maxLineGap=5)
    #cvu.draw_lines(linesP[0],debug_frame)
    # if linesP is not None:
    #     for i in range(0, len(linesP)):
    #             l = linesP[i][0]
    #             cv2.line(debug_frame, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv2.LINE_AA)
    #             cv2.putText(debug_frame, str(cvu.get_orientation_gl(l)), (l[0], l[1] + 10),cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
    
    line_buffer=[None,None,None]
    for i in range(0,len(masks)):
        mask=masks[i]
        lines = cv2.HoughLinesP(cvu.roi(edges,[mask]),1, np.pi/180, threshold=100, minLineLength=20, maxLineGap=5)
        bundler = cvu.HoughBundler(min_distance=40,min_angle=5)
        lines = bundler.process_lines(lines,(i))
       
        if lines !=[]:
            walls[i]=1
            lines=cvu.closest_to_center(lines)
            line_buffer[i]=lines[0]
        else:
            walls[i]=0
            #create a dummy line
            if(i==0):
                lines=[[0,480,120,0]]
                line_buffer[i]=[0,480,120,0]
            if(i==1):
                lines=[[0,200,640,300]]
                line_buffer[i]=[0,200,640,300]
            elif(i==2):
                lines=[[640,480,520,0]]
                line_buffer[i]=[640,480,520,0]
        #cvu.draw_lines(lines,frame) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!debug
   
    p1=(walls.count(1)==0) # no line segement is deteced
    p2=(walls.count(1)==1) # one line segment detected
    p3=(walls.count(1)==2) # two line segments detected
    p4=(walls.count(1)==3) # three line segments detected
    if p3:
        indices = [i for i, num in enumerate(walls) if num == 1]
        s1,s2=indices
        t1,t2=cvu.find_vanishing_point([line_buffer[s1], line_buffer[s2]])
        p5=not(cvu.is_point_inside_region(t1,t2,640,480)) # do the vanishing points lie inside the image
        p6=cvu.do_line_segments_intersect(line_buffer[s1], line_buffer[s2]) # the line segments intersect
    else:
        p5=0
        p6=0
    next_state=state(p1,p2,p3,p4,p5,p6)
    action_taken=action(current_state,next_state)
    x_m,x_v,linear_vel=points_from_action(action_taken,line_buffer)
    #print(offset)
    # h=2.12
    # camera_tilt=36*np.pi/180
    # focal_length=8.247
    # sensor_width = 36
    # sensor_height = 24
    # horizontal_scale = sensor_width / (2 * focal_length)
    # k1=horizontal_scale*camera_tilt/np.cos(camera_tilt)
    # k2=-horizontal_scale*focal_length*np.sin(camera_tilt)/h
    # k3=horizontal_scale*focal_length*np.cos(camera_tilt)
    # kp=10
   
    # angular_vel = -(k1/(k1*k3+x_m*x_v))*(-(k2/k1)*linear_vel*x_v-kp*x_m)
    angular_vel=((x_m+x_v)/2-320)/2 + (x_m-x_v)/15
   
    # cv2.circle(frame, (int(x_m),240), 4, (0,255,0), 10) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!debug
    # cv2.circle(frame, (int(x_v),240), 4, (255,0,0), 10)
    # cvu.__draw_label(frame, 'angular vel = %f' % (angular_vel), (20,20), (0,50,255))
    # cvu.__draw_label(frame, 'walls = [%d , %d, %d]' % tuple(walls), (20,40), (0,50,255))
    # cvu.__draw_label(frame, 'next_state = %s (%s)' % (decode_state(next_state),next_state), (20,60), (0,50,255))
    # cvu.__draw_label(frame, 'current_state = %s (%s)' % (decode_state(current_state),current_state), (20,80), (0,50,255))
    
    return next_state,linear_vel,angular_vel,frame,debug_frame
    
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