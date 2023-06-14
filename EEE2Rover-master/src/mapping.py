import numpy as np
import cv2
import matplotlib.pyplot as plt
import random
import math

def create_fixed_size_image(width, height, color=(0, 0, 0)):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = color
    return image

def overlay_image(background, foreground, position, size, rotation):
    # Resize the foreground image to the desired size
    foreground_resized = cv2.resize(foreground, size)
    
    # Rotate the foreground image
    foreground_rotated = rotate_image(foreground_resized, rotation)
    
    # Get the dimensions of the rotated foreground image
    rows, cols, _ = foreground_rotated.shape
    
    # Calculate the position for placing the foreground image onto the background
    x, y = position[0] - int(cols / 2), position[1] - int(rows / 2)
    
    # Create a copy of the background image
    result = background.copy()
    
    # Overlay the rotated foreground image onto the background
    for i in range(rows):
        for j in range(cols):
            if all(foreground_rotated[i, j] > 0):  # Check if the pixel is non-black
                result[y + i, x + j] = foreground_rotated[i, j]
    
    return result

def rotate_image(image, angle):
    rows, cols, _ = image.shape
    M = cv2.getRotationMatrix2D((cols/2, rows/2), angle, 1)
    rotated_image = cv2.warpAffine(image, M, (cols, rows))
    return rotated_image

def show_map(map):  
    plt.imshow(map)
    plt.show()