import cv2
import numpy as np
import math
import calibration_utils
import perspective_utils
video = cv2.VideoCapture("maze_view.mp4")
        
kernel = np.ones((5,5),np.uint8)    
   
vertices= np.array([[0,400],[0,300],[640,300],[640,400],
                         ], np.int32)  
   
forward_mask= np.array([[0,350],[0,300],[640,300],[640,350],
                         ], np.int32)  
left_mask = np.array([[0,480],[0,0],[320,0],[320,480],
                         ], np.int32)  
right_mask = np.array([[320,480],[320,0],[640,0],[640,480],
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
                if (np.arctan(abs(line_slope-group_slope)) < threshold_angle)&(np.linalg.norm((group_center-line_center),ord=1)<threshold_dist):
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

def apply_brightness_contrast(input_img, brightness = 0, contrast = 0):
    
    if brightness != 0:
        if brightness > 0:
            shadow = brightness
            highlight = 255
        else:
            shadow = 0
            highlight = 255 + brightness
        alpha_b = (highlight - shadow)/255
        gamma_b = shadow
        
        buf = cv2.addWeighted(input_img, alpha_b, input_img, 0, gamma_b)
    else:
        buf = input_img.copy()
    
    if contrast != 0:
        f = 131*(contrast + 127)/(127*(131-contrast))
        alpha_c = f
        gamma_c = 127*(1-f)
        
        buf = cv2.addWeighted(buf, alpha_c, buf, 0, gamma_c)

    return buf

class HoughBundler:     
    def __init__(self,min_distance=5,min_angle=2):
        self.min_distance = min_distance
        self.min_angle = min_angle
    
    def get_orientation(self, line):
        orientation = math.atan2(abs((line[3] - line[1])), abs((line[2] - line[0])))
        return math.degrees(orientation)

    def check_is_line_different(self, line_1, groups, min_distance_to_merge, min_angle_to_merge):
        for group in groups:
            for line_2 in group:
                if self.get_distance(line_2, line_1) < min_distance_to_merge:
                    orientation_1 = self.get_orientation(line_1)
                    orientation_2 = self.get_orientation(line_2)
                    if abs(orientation_1 - orientation_2) < min_angle_to_merge:
                        group.append(line_1)
                        return False
        return True

    def distance_point_to_line(self, point, line):
        px, py = point
        x1, y1, x2, y2 = line

        def line_magnitude(x1, y1, x2, y2):
            line_magnitude = math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))
            return line_magnitude

        lmag = line_magnitude(x1, y1, x2, y2)
        if lmag < 0.00000001:
            distance_point_to_line = 9999
            return distance_point_to_line

        u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
        u = u1 / (lmag * lmag)

        if (u < 0.00001) or (u > 1):
            #// closest point does not fall within the line segment, take the shorter distance
            #// to an endpoint
            ix = line_magnitude(px, py, x1, y1)
            iy = line_magnitude(px, py, x2, y2)
            if ix > iy:
                distance_point_to_line = iy
            else:
                distance_point_to_line = ix
        else:
            # Intersecting point is on the line, use the formula
            ix = x1 + u * (x2 - x1)
            iy = y1 + u * (y2 - y1)
            distance_point_to_line = line_magnitude(px, py, ix, iy)

        return distance_point_to_line

    def get_distance(self, a_line, b_line):
        dist1 = self.distance_point_to_line(a_line[:2], b_line)
        dist2 = self.distance_point_to_line(a_line[2:], b_line)
        dist3 = self.distance_point_to_line(b_line[:2], a_line)
        dist4 = self.distance_point_to_line(b_line[2:], a_line)

        return min(dist1, dist2, dist3, dist4)

    def merge_lines_into_groups(self, lines):
        groups = []  # all lines groups are here
        # first line will create new group every time
        groups.append([lines[0]])
        # if line is different from existing gropus, create a new group
        for line_new in lines[1:]:
            if self.check_is_line_different(line_new, groups, self.min_distance, self.min_angle):
                groups.append([line_new])

        return groups

    def merge_line_segments(self, lines):
        # orientation = self.get_orientation(lines[0])
      
        # if(len(lines) == 1):
        #     return np.block([[lines[0][:2], lines[0][2:]]])

        # points = []
        # for line in lines:
            
        #     points.append(line[:2])
        #     points.append(line[2:])
        # if 45 < orientation <= 90:
        #     #sort by y
        #     points = sorted(points, key=lambda point: point[1])
        # else:
        #     #sort by x
        #     points = sorted(points, key=lambda point: point[0])
      
       # return np.block([[points[0],points[-1]]])
      
       return (np.mean(lines, axis=0, dtype=np.int32))

    def process_lines(self, lines,vert):
        lines_horizontal  = []
        lines_vertical  = []
        if lines is not None:
            
            for line_i in [l[0] for l in lines]:
                orientation = self.get_orientation(line_i)
                # if vertical
                if 45 < orientation <= 90:
                    lines_vertical.append(line_i)
                else:
                    lines_horizontal.append(line_i)

        lines_vertical  = sorted(lines_vertical , key=lambda line: line[1])
        lines_horizontal  = sorted(lines_horizontal , key=lambda line: line[0])
        merged_lines_all = []

        if vert:
            target_lines=lines_vertical
        else:
            target_lines=lines_horizontal
        # for each cluster in vertical and horizantal lines leave only one line
        for i in [target_lines]:
            if len(i) > 0:
                groups = self.merge_lines_into_groups(i)
                merged_lines = []
                for group in groups:
                    merged_lines.append(self.merge_line_segments(group))
                merged_lines_all.extend(merged_lines)
        
    
        
        return (merged_lines_all)

# Usage:



def roi(img, vertices):
    #blank mask:
    mask = np.zeros_like(img)
    # fill the mask
    cv2.fillPoly(mask, vertices, 255)
    # now only show the area that is the mask
    masked = cv2.bitwise_and(img, mask)
    return masked

def __draw_label(img, text, pos, bg_color):
   font_face = cv2.FONT_HERSHEY_SIMPLEX
   scale = 0.4
   color = (255, 255, 255)
   thickness = cv2.FILLED
   margin = 2
   txt_size = cv2.getTextSize(text, font_face, scale, thickness)

   end_x = pos[0] + txt_size[0][0] + margin
   end_y = pos[1] - txt_size[0][1] - margin
    
   #cv2.rectangle(img, pos, (end_x, end_y), bg_color, thickness)
   cv2.putText(img, text, pos, font_face, scale, color, 1, cv2.LINE_AA)
# Adjust this threshold to control line grouping
threshold_theta =1000000000000
threshold_delta = 100
#count for exporting frames
count =0

# Specify size on horizontal axis
def draw_lines(lines,frame):
     if lines is not None:
       # lines = bundler.process_lines(lines)
        for line in lines:
            
            x1, y1, x2, y2 = line
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

while True:
    
    ret, frame = video.read()
    
   
    if not ret:
        video = cv2.VideoCapture("maze_view.mp4")
        continue
    frame = cv2.resize(frame, (640, 480))
   
  
 
    edges = cv2.Canny(frame, 150, 255)
    edges=roi(edges,[vertices])
    #edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    masks=[left_mask,forward_mask,right_mask]
    walls=[0,0,0]
    for i in range(0,len(masks)):
        mask=masks[i]
        lines = cv2.HoughLinesP(roi(edges,[mask]), 1, np.pi/180, threshold=50, minLineLength=20 , maxLineGap=10)
        bundler = HoughBundler(min_distance=20,min_angle=10)
        lines = bundler.process_lines(lines,(i!=1))
        if lines != []:
            walls[i]=1

        else:
            walls[i]=0
        draw_lines(lines,frame)
    
    __draw_label(frame, 'offset = %d' % -0.1, (20,20), (255,255,255))
    __draw_label(frame, 'walls = [%d , %d, %d]' % tuple(walls), (20,40), (255,255,255))
    cv2.imshow("edges", edges)
    cv2.imshow("frame", frame)
    key = cv2.waitKey(25)
    if key == 27:
        break
    count += 1
video.release()
cv2.destroyAllWindows()