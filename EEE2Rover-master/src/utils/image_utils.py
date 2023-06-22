
import numpy as np
import cv2
import matplotlib.pyplot as plt 

def uv2xy(frame, theta, phi,alpha,r_c):
    x_c,y_c,z_c=r_c
    TARGET_W =10*100
    TARGET_H =6*100
 
    f=3.6e-3 
    mu=2.2e-6
    mv=2.2e-6
    input_h, input_w,_ = frame.shape
    output=np.zeros((TARGET_W,TARGET_H,3),dtype=np.uint8)
    #Other matrix
    M = np.array([[np.sin(theta), -np.cos(theta), 0],
                  [np.cos(phi)*np.cos(theta),np.cos(phi)*np.sin(theta), np.sin(phi)],
                  [np.sin(phi)*np.cos(theta),np.sin(phi)*np.sin(theta), -np.cos(phi)]])
    
    
    K=np.array([[f/mu,0,mu*TARGET_W/2],
                [0,f/mv,mv*TARGET_H/2],
                [0,0,1]])
   
    # for y in range(0,TARGET_H):
    #     for x in range(0,TARGET_W):
    #         x*=0.02
    #         y*=0.02
    #         u,v,w= K @ M @ np.array([x,y,0-z_c]).T
    #         u=u/w
    #         v=v/w
    #         x/=0.02
    #         y/=0.02
           
    #         if u>=0 and u<input_w and v>=0 and v<input_h:
    #             #print("{u,v}",np.floor(u).astype(int),",",np.floor(v).astype(int),"{x,y}",np.floor(x).astype(int),",",np.floor(y).astype(int),"RGB",frame[np.floor(v).astype(int),np.floor(u).astype(int)])
    #             output[np.floor(x).astype(int),np.floor(y).astype(int)]=frame[np.floor(u).astype(int),np.floor(v).astype(int),]
   
    output=cv2.warpPerspective(frame, K@M.T, (TARGET_H,TARGET_W))

    return output


frame=cv2.imread("MVM.png")

#uv2xy params



z_c=1# z axis coordinate of camera
x_c=0
y_c=0
alpha=np.radians(45) #camera fov in rad
phi=np.rad2deg(80) #camera elevation relative to x-y plane in rad
theta=np.rad2deg(45)#camera azimuth angle in rad based on x axis direction
r_c=(x_c,y_c,z_c)
cv2.imshow("ada",frame)
cv2.imshow("sdad",uv2xy(frame, theta, phi, alpha, r_c))
print("done")
#plt.show()
cv2.waitKey(0)
cv2.destroyAllWindows()



   
    # M = K @ T @ R
   
    # for y in range(TARGET_H):
    #     for x in range(TARGET_W):
          
    #         u,v,_= M @ ( np.array([x,y,0,1]).T)/z_c
    #         if u>=0 and u<input_w and v>=0 and v<input_h:
    #             output[y,x]=frame[int(v),int(u)]
   

#camera instrinsics+extrinsics
    # R=np.array([[1,0,0,0],
    #             [0,np.cos(theta),-np.sin(theta),0],
    #             [0,np.sin(theta),np.cos(theta),0],
    #             [0,0,0,1]])
    # T=np.array([[1,0,0,0],
    #             [0,1,0,0],
    #             [0,0,1,-z_c/np.sin(theta)],
    #             [0,0,0,1]])
    # K=np.array([[f*ku,s,u0,0],[0,f*kv,v0,0],[0,0,1,0]])
    # s=1000/(np.tan(alpha/2))
    