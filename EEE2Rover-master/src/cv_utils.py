import cv2
import numpy as np
import math
import calibration_utils
import perspective_utils

def __draw_label(img, text, pos, bg_color):
   font_face = cv2.FONT_HERSHEY_SIMPLEX
   scale = 0.6
   color = (255, 255, 255)
   thickness = cv2.FILLED
   margin = 2
   txt_size = cv2.getTextSize(text, font_face, scale, thickness)

   end_x = pos[0] + txt_size[0][0] + margin
   end_y = pos[1] - txt_size[0][1] - margin
    
   #cv2.rectangle(img, pos, (end_x, end_y), bg_color, thickness)
   cv2.putText(img, text, pos, font_face, scale, color, 1, cv2.LINE_AA)

def roi(img, vertices):
    #blank mask:
    mask = np.zeros_like(img)
    # fill the mask
    cv2.fillPoly(mask, vertices, 255)
    # now only show the area that is the mask
    masked = cv2.bitwise_and(img, mask)
    return masked
def draw_lines(lines,frame):
     if lines is not None:
       # lines = bundler.process_lines(lines)
        for line in lines:
            if line is not None:
                x1, y1, x2, y2 = line
                cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
def draw_grid(img, grid_shape, color=(0, 255, 0), thickness=1):
    h, w, _ = img.shape
    rows, cols = grid_shape
    dy, dx = h / rows, w / cols

    # draw vertical lines
    for x in np.linspace(start=dx, stop=w-dx, num=cols-1):
        x = int(round(x))
        cv2.line(img, (x, 0), (x, h), color=color, thickness=thickness)

    # draw horizontal lines
    for y in np.linspace(start=dy, stop=h-dy, num=rows-1):
        y = int(round(y))
        cv2.line(img, (0, y), (w, y), color=color, thickness=thickness)

    return img
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
def GetVanishingPoint(Lines):
   #find the midpoint of the lines

   x1, y1, x2, y2 = Lines[0]
   x1c, y1c, x2c, y2c = Lines[1]
   x=(x1+x2+x1c+x2c)/4
   return [int(x),240]
def closest_to_center(lines):
    if lines !=[]:
        closest_line = lines[0]
        for line in lines:
            x1, y1, x2, y2 = line
            x1c, y1c, x2c, y2c = closest_line
            if abs(320-(x1+x2)/2) <abs(320-(x1c+x2c)/2) :
                closest_line=line
    return [closest_line]
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

    def merge_line_segments(self, lines,):
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

    def process_lines(self, lines, align):
        lines_horizontal  = []
        lines_vertical  = []
        if lines is not None:
            
            for line_i in [l[0] for l in lines]:
                orientation = self.get_orientation(line_i)
                # if vertical
                if 25 < orientation <= 90:
                    lines_vertical.append(line_i)
                else:
                    lines_horizontal.append(line_i)

        lines_vertical  = sorted(lines_vertical , key=lambda line: line[1])
        lines_horizontal  = sorted(lines_horizontal , key=lambda line: line[0])
        merged_lines_all = []
        if align == 1:
            target_lines = lines_vertical
        else:
            target_lines = lines_horizontal
        # for each cluster in vertical and horizantal lines leave only one line
        for i in [target_lines]:
            if len(i) > 0:
                groups = self.merge_lines_into_groups(i)
                merged_lines = []
                for group in groups:
                    merged_lines.append(self.merge_line_segments(group))
                merged_lines_all.extend(merged_lines)
        
    
        
        return (merged_lines_all)