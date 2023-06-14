import numpy as np
from PIL.Image import core as _imaging
from PIL import Image

roverState = 0 #can send commands to rover when roverState = 1
x_prev = 0
y_prev = 0

def decode_frame(chunks, width=640, height=480):
    num_chunks = len(chunks) // 8

    # Create a blank image
    img = np.zeros([height, width, 3], dtype=np.uint8)
    img.fill(0)  # fill with white

    for i in range(num_chunks - 1, -1, -1):
        chunk = chunks[i * 8 : (i * 8) + 8]
        binary = bin(int(chunk, 16))[2:].zfill(32)
        #11 12 13 14 15 16 17 18 
        #new_frame = int(binary[0:1], 2)
        x_start = int(binary[1:11], 2)
        y_start = int(binary[11:19], 2)
        run_length = int(binary[19:32], 2)

        # if new_frame:
        #     img = np.zeros([height, width, 3], dtype=np.uint8)
        #     img.fill(255)  # reset to white

        if x_start + run_length <= width:
            for x in range(x_start, x_start + run_length):
                # Set the pixel to black
                img[y_start, x] = [255, 255, 255]  # RGB


    # Convert numpy array to Image
    #img_pil = Image.fromarray(img)
    #img_cv = np.array(img_pil)
    #img_cv = img_cv[:, :, ::-1].copy() 
    return img

def decode_position(posdata): #
    global roverState
    roverState = posdata[0]

    angleStr = posdata[1:6]
    whole = int(angleStr[0:3])
    fraction = int(angleStr[3:5]) / 100.0
    angle = np.radians(whole + fraction)

    displacementStr = posdata[6:11]
    whole = int(displacementStr[0:3])
    fraction = int(displacementStr[3:5]) / 100.0
    displacement = whole + fraction

    deltax = int(displacement * np.cos(np.pi/2 - angle))
    deltay = int(displacement * np.sin(np.pi/2 - angle))
    global x_prev, y_prev
    x_prev += deltax
    y_prev += deltay
    return x_prev, y_prev








    



# # Convert PIL Image to OpenCV image (numpy array)


# # Convert RGB to BGR 
# 

# Now you can use 'img_cv' with OpenCV functions
# cv2.imshow('image', img_cv)
# cv2.waitKey(0)
# cv2.destroyAllWindows()