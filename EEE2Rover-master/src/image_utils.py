
import numpy as np
import cv2
import matplotlib.pyplot as plt 

def uv2xy(frame, theta, phi, r_c):
    x_c,y_c,z_c=r_c
    TARGET_W =3000
    TARGET_H =3000
    input_h, input_w = frame.shape
    output=np.zeros((TARGET_H,TARGET_W,3),dtype=np.uint8)
    # Define transformation matrix
    M = np.array([[np.sin(theta), -np.cos(theta), 0],
                  [np.cos(phi)*np.cos(theta),np.cos(phi)*np.sin(theta), np.sin(phi)],
                  [np.sin(phi)*np.cos(theta),np.sin(phi)*np.sin(theta), -np.cos(phi)]])
    
    
    for y in range(TARGET_H):
        for x in range(TARGET_W):
            z=0
            u,v,w= M @ np.array([x-x_c,y-y_c,z-z_c]).T
            u=-z_c*u/w
            v=-z_c*v/w
            if u>=0 and u<input_w and v>=0 and v<input_h:
                output[y,x]=frame[int(v),int(u)]
    
    return output


frame=plt.imread("2013mef-H0M.jpg")

#uv2xy params


#2000p = 6mm
#58mm
z_c=(53/6)*13# z axis coordinate of camera
x_c=0
y_c=0
alpha=np.radians(40) #camera fov in rad
phi=np.rad2deg(75) #camera elevation relative to x-y plane in rad
theta=-np.rad2deg(45)#camera azimuth angle in rad based on x axis direction
r_c=(0,0,z_c)
plt.imshow(frame)
plt.imshow(uv2xy(frame, theta, phi, r_c))
print("done")
plt.show()
# cv2.destroyAllWindows()