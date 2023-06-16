import asyncio
import websockets
import cv2
import numpy as np
import processing
import json
import mapping
import decode

ip = "ws://192.168.4.1/"
map = np.zeros((400, 200, 3), dtype=np.uint8)  # Create a black image
goal=(200,100)
positions = []  # Example positions
brush_size = 5
brush_color = (0, 0, 255)  # Blue color
current_state=0

async def receive_camera_frame(websocket):
    async with websockets.connect(ip+'ws') as websocket:
        print("connected")
        global current_state
        global positions
        global map
        
        while True:
            response = await websocket.recv()
            frame=decode.decode_frame(response)
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            #x_y_position=decode.decode_position(response[-11:])
            current_state,linear_vel,angular_vel,img,filtered_img=processing.analyse_frame(frame,current_state, (0,0), 0)
            cv2.imshow('Debug', filtered_img)
            cv2.imshow("Img", img)
            #map= mapping.trace_brush(map, positions, brush_size, brush_color)
            cv2.imshow('map', map)
            await websocket.send(str(linear_vel)+","+str(angular_vel))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()
        

async def run_test():
    async with websockets.connect(ip+'ws') as websocket:
        while True:
            await receive_camera_frame(websocket)
asyncio.run(run_test())
