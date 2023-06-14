import asyncio
import websockets
import cv2
import numpy as np
import processing
current_state=0
async def receive_camera_frame(websocket, path):
    global current_state
    while True:
        frame_bytes = await websocket.recv()
        frame_np = np.frombuffer(frame_bytes, dtype=np.uint8)
        frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)
        current_state,linear_vel,angular_vel,img,filtered_img=processing.analyse_frame(frame,current_state)
        cv2.imshow('Debug', filtered_img)
        cv2.imshow("Img", img)
        await websocket.send(str(linear_vel)+","+str(angular_vel))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
start_server = websockets.serve(receive_camera_frame, '0.0.0.0', 8000)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
