import asyncio
import websockets
import cv2
import numpy as np
import processing
import json
import mapping
# Example usage
map = np.zeros((1000, 500, 3), dtype=np.uint8)  # Create a black image
positions = []  # Example positions
brush_size = 15
brush_color = (255, 255, 255)  # Blue color



# Display the traced image


current_state=0
async def receive_camera_frame(websocket, path):
    global current_state
    global positions
    global map
    while True:
        data = await websocket.recv()
        
        # Parse the JSON data
        dataObject = json.loads(data)
        print(dataObject)
        # Extract position data
        position = dataObject['position']
        x = 50*position['x']
        z = 50*position['z']
        x_z_position = (int(x), int(z))
        positions.append(x_z_position)
        # Extract rotation data
        rotation = dataObject['rotation']
        y_rotation = rotation['y']

        # Process the position and rotation data
        print(f"Received position: ({x}, {z})")
        print(f"Received y-axis rotation: {y_rotation}")
        frame_bytes = await websocket.recv()
        frame_np = np.frombuffer(frame_bytes, dtype=np.uint8)
        frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)
        current_state,linear_vel,angular_vel,img,filtered_img=processing.analyse_frame(frame,current_state, x_z_position, y_rotation)
        cv2.imshow('Debug', filtered_img)
        cv2.imshow("Img", img)
        map= mapping.trace_brush(map, positions, brush_size, brush_color)
        cv2.imshow('map', map)
        await websocket.send(str(linear_vel)+","+str(angular_vel))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
start_server = websockets.serve(receive_camera_frame, '0.0.0.0', 8000)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
