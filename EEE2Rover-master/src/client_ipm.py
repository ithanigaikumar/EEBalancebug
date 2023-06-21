import asyncio
import websockets
import cv2
import numpy as np
import processing
import json
import mapping
import decode
import keyboard
ip = "ws://192.168.128.85/"
#map = np.zeros((400, 200, 3), dtype=np.uint8)  # Create a black image
#goal=(200,100)
#positions = []  # Example positions
#brush_size = 5
#brush_color = (0, 0, 255)  # Blue color
#current_state=0
#current_position=[0,0]

async def receive_camera_frame(websocket):
    print("connected")
    global current_state
    global positions
    global map

    while True:
        count=0
        response = await websocket.recv()
        frame=decode.decode_frame(response)
        #frame=cv2.rotate(frame,cv2.ROTATE_180)
        #ds = float(await websocket.recv())
        #dtheta = float(await websocket.recv())
        #print("ds:"+str(ds),"dtheta:",str(dtheta))
        
        cv2.imshow("frame",frame)
        if keyboard.is_pressed('s'):
            cv2.imwrite("frame"+count+".png")
            count+=1

        if keyboard.is_pressed('w'):
            await websocket.send(str(1)+","+"0")
            await asyncio.sleep(1)
            await websocket.send(str(0)+","+str(Z))
        if keyboard.is_pressed('a'):
                angular_vel=-15
                await websocket.send(str(0)+","+str(angular_vel))
                await asyncio.sleep(1)
                angular_vel=0
                await websocket.send(str(0)+","+str(angular_vel))
        if keyboard.is_pressed('d'):
                angular_vel=-15
                await websocket.send(str(0)+","+str(angular_vel))
                await asyncio.sleep(1)
                angular_vel=0
                await websocket.send(str(0)+","+str(angular_vel))
        
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
        

async def run_test():
    async with websockets.connect(ip+'ws',ping_timeout=None) as websocket:
        while True:
            await receive_camera_frame(websocket)
asyncio.run(run_test())


