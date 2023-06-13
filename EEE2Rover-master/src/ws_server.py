import asyncio
import websockets
import cv2
import numpy as np

async def receive_camera_frame(websocket, path):
    while True:
        frame_bytes = await websocket.recv()

        frame_np = np.frombuffer(frame_bytes, dtype=np.uint8)
        frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)

        # Process the frame as needed
        cv2.imshow('Camera Frame', frame)
        cv2.waitKey(1)

start_server = websockets.serve(receive_camera_frame, '0.0.0.0', 8000)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
