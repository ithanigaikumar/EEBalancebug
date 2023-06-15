import numpy as np
import cv2
import matplotlib.pyplot as plt
import random
import math

def trace_brush(image, positions, brush_size, brush_color):
    # Create a copy of the image
    traced_image = image.copy()

    # Iterate over the positions and draw circles at each position
    for position in positions:
        cv2.circle(traced_image, position, brush_size, brush_color, -1)

    return traced_image