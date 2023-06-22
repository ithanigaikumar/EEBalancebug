
import cv2
image_paths=['MVL.png','MVM.png','MVR.png']
# initialized a list of images
imgs = []
  
for i in range(len(image_paths)):
    imgs.append(cv2.imread(image_paths[i]))
    imgs[i]=cv2.resize(imgs[i],(0,0),fx=0.5,fy=0.5)
    # this is optional if your input images isn't too large
    # you don't need to scale down the image
    # in my case the input images are of dimensions 3000x1200
    # and due to this the resultant image won't fit the screen
    # scaling down the images 
# showing the original pictures
cv2.imshow('1',imgs[0])
cv2.imshow('2',imgs[1])
cv2.imshow('3',imgs[2])
  
stitcher=cv2.Stitcher.create(cv2.Stitcher_PANORAMA)
status,panorama = stitcher.stitch(imgs) 
cv2.imshow("panorama", panorama)
cv2.waitKey(0)
cv2.destroyAllWindows