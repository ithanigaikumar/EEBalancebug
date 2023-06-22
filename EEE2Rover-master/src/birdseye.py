import cv2
import os
import numpy as np
import perspective_utils


def birdseye(img):

    img=cv2.resize(img,(640, 480))
    # Ratio 8m width to 6m height (4:3)
    width = 530
    height = 246
    d2= 39


    #Input points are read from the corners drawn in the reduced size screenshot provided
    inputpts = np.float32([[0, 240],[640,240],[0, 480], [640, 480]])
    outputpts = np.float32([[width//2-265,0], [width//2+265, 0], [width//2-39, height-33], [width//2+39, height-33]])



    m = cv2.getPerspectiveTransform(inputpts, outputpts)
    outimg = cv2.warpPerspective(img, m, (width, height), cv2.INTER_LINEAR)
    return outimg


# imagepath = "hhello.jpg"
# img = cv2.imread(imagepath)
# outimg=birdseye(img)
# cv2.imshow("hell",img)
# #cv2.imshow("hello",img[240:480, 0:640])
# cv2.imshow('Result', outimg)
# k = cv2.waitKey(0)