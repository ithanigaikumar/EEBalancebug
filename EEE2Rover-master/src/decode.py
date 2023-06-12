import numpy as np
from PIL.Image import core as _imaging
from PIL import Image


def decode_frame(chunks, width=640, height=480):
    num_chunks = len(chunks) // 8

    # Create a blank image
    img = np.zeros([height, width, 3], dtype=np.uint8)
    img.fill(0)  # fill with white

    for i in range(num_chunks - 1, -1, -1):
        chunk = chunks[i * 8 : (i * 8) + 8]
        binary = bin(int(chunk, 16))[2:].zfill(32)

        #new_frame = int(binary[0:1], 2)
        x_start = int(binary[1:11], 2)
        y_start = int(binary[11:19], 2)
        chunk_length = int(binary[19:32], 2)

        # if new_frame:
        #     img = np.zeros([height, width, 3], dtype=np.uint8)
        #     img.fill(255)  # reset to white

        if x_start + chunk_length <= width:
            for x in range(x_start, x_start + chunk_length):
                # Set the pixel to black
                img[y_start, x] = [255, 255, 255]  # RGB

    # Convert numpy array to Image
    #img_pil = Image.fromarray(img)
    #img_cv = np.array(img_pil)
    #img_cv = img_cv[:, :, ::-1].copy() 
    return img



# # Convert PIL Image to OpenCV image (numpy array)


# # Convert RGB to BGR 
# 

# Now you can use 'img_cv' with OpenCV functions
# cv2.imshow('image', img_cv)
# cv2.waitKey(0)
# cv2.destroyAllWindows()