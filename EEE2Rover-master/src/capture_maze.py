import asyncio
import websockets
import cv2
import numpy as np
import processing
import json
import mapping
import decode
import os

if not os.path.exists('images'):
    os.makedirs('images')

ip = "ws://192.168.4.1/"
map = np.zeros((400, 200, 3), dtype=np.uint8)  # Create a black image
goal=(200,100)
positions = []  # Example positions
brush_size = 5
brush_color = (0, 0, 255)  # Blue color
current_state=0
turns = 0  # Counter for turns
images = []  # List to store images

# async def receive_camera_frame(websocket):
#     global current_state
#     global positions
#     global map
#     global turns
#     global images
#     retry_count = 0
#     max_retries = 6  # Set a max limit for retries to prevent endless loop

#     while turns < 6:
#         try:
#             # If connection is not established or has been lost, reestablish it
#             if websocket.closed:
#                 cv2.destroyAllWindows()
#                 websocket = await websockets.connect(ip+'ws')
#                 print("Connected")
            

#             # Control the angular and linear velocity
#             if turns < 6:
#                 angular_vel = 30
#                 linear_vel = 0
#             else:
#                 angular_vel, linear_vel = 0, 0
                
            
#             # Send control commands
#             await websocket.send(str(linear_vel) + "," + str(angular_vel))
#             await asyncio.sleep(1)  
#             await websocket.send(str(0) + "," + str(0))
#             await asyncio.sleep(1)  
#             # response = await websocket.recv()
#             # frame = decode.decode_frame(response)
#             # #current_state, linear_vel, angular_vel, img, filtered_img = processing.analyse_frame(frame, current_state, (0, 0), 0)
#             # images.append(frame)  # Add image to list
#             # cv2.imwrite(f'images/image_{turns}_.jpg', frame)
#             # # # If rotation completed, then receive frame
#             if turns > 0:
#                 response = await websocket.recv()
#                 frame = decode.decode_frame(response)
#                 #current_state, linear_vel, angular_vel, img, filtered_img = processing.analyse_frame(frame, current_state, (0, 0), 0)
#                 images.append(frame)  # Add image to list
#                 #cv2.imshow("frame",frame)
#                 cv2.imwrite(f'images/image_{turns}.jpg', frame)

#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break

#             retry_count = 0  # reset retry count on successful data receipt
#             turns += 1
            

#         except (websockets.exceptions.ConnectionClosedError, ConnectionResetError) as e:
#             print(f'Error occurred: {e}. Retrying...')
#             retry_count += 1
#             if retry_count > max_retries:
#                 print("Reached maximum retries. Exiting...")
#                 cv2.destroyAllWindows()
#                 break
#             await asyncio.sleep(1)  # Wait for a second before retrying
#     print("Done")
#     #stitcher=cv2.Stitcher.create(cv2.Stitcher_PANORAMA)
#     #status,panorama=stitcher.stitch(frame)
#     #cv2.imwrite('images/panorama.png',panorama)
# async def run_test():
#     async with websockets.connect(ip+'ws') as websocket:
#         while True:
#             await receive_camera_frame(websocket)
#             if turns >= 6:  # If 6 turns completed, then break the loop
#                 break
# asyncio.run(run_test())

async def receive_camera_frame(websocket):
    global current_state
    global positions
    global map
    global turns
    global images

    linear_vel = 0


    response = await websocket.recv()
    frame = decode.decode_frame(response)
    #current_state, linear_vel, angular_vel, img, filtered_img = processing.analyse_frame(frame, current_state, (0, 0), 0)
    images.append(frame)  # Add image to list
    #cv2.imshow("frame",frame)
    cv2.imwrite(f'images/image_{0}.jpg', frame)

    await websocket.send(str(linear_vel) + "," + str(40))
    await asyncio.sleep(1)
    await websocket.send(str(linear_vel) + "," + str(0))
    await asyncio.sleep(2)
    response = await websocket.recv()
    frame = decode.decode_frame(response)
    #current_state, linear_vel, angular_vel, img, filtered_img = processing.analyse_frame(frame, current_state, (0, 0), 0)
    images.append(frame)  # Add image to list
    #cv2.imshow("frame",frame)
    cv2.imwrite(f'images/image_{1}.jpg', frame)

    await websocket.send(str(linear_vel) + "," + str(40))
    await asyncio.sleep(1)
    await websocket.send(str(linear_vel) + "," + str(0))
    await asyncio.sleep(2)
    response = await websocket.recv()
    frame = decode.decode_frame(response)
    #current_state, linear_vel, angular_vel, img, filtered_img = processing.analyse_frame(frame, current_state, (0, 0), 0)
    images.append(frame)  # Add image to list
    #cv2.imshow("frame",frame)
    cv2.imwrite(f'images/image_{2}.jpg', frame)

    await websocket.send(str(linear_vel) + "," + str(40))
    await asyncio.sleep(1)
    await websocket.send(str(linear_vel) + "," + str(0))
    await asyncio.sleep(2)
    response = await websocket.recv()
    frame = decode.decode_frame(response)
    #current_state, linear_vel, angular_vel, img, filtered_img = processing.analyse_frame(frame, current_state, (0, 0), 0)
    images.append(frame)  # Add image to list
    #cv2.imshow("frame",frame)
    cv2.imwrite(f'images/image_{3}.jpg', frame)

    await websocket.send(str(linear_vel) + "," + str(40))
    await asyncio.sleep(1)
    await websocket.send(str(linear_vel) + "," + str(0))
    await asyncio.sleep(2)
    response = await websocket.recv()
    frame = decode.decode_frame(response)
    #current_state, linear_vel, angular_vel, img, filtered_img = processing.analyse_frame(frame, current_state, (0, 0), 0)
    images.append(frame)  # Add image to list
    #cv2.imshow("frame",frame)
    cv2.imwrite(f'images/image_{4}.jpg', frame)

    await websocket.send(str(linear_vel) + "," + str(40))
    await asyncio.sleep(1)
    await websocket.send(str(linear_vel) + "," + str(0))
    await asyncio.sleep(2)
    response = await websocket.recv()
    frame = decode.decode_frame(response)
    #current_state, linear_vel, angular_vel, img, filtered_img = processing.analyse_frame(frame, current_state, (0, 0), 0)
    images.append(frame)  # Add image to list
    #cv2.imshow("frame",frame)
    cv2.imwrite(f'images/image_{5}.jpg', frame)

    await websocket.send(str(linear_vel) + "," + str(40))
    await asyncio.sleep(1)
    await websocket.send(str(linear_vel) + "," + str(0))
    await asyncio.sleep(2)
    response = await websocket.recv()
    frame = decode.decode_frame(response)
    #current_state, linear_vel, angular_vel, img, filtered_img = processing.analyse_frame(frame, current_state, (0, 0), 0)
    images.append(frame)  # Add image to list
    #cv2.imshow("frame",frame)
    cv2.imwrite(f'images/image_{6}.jpg', frame)

    await websocket.send(str(linear_vel) + "," + str(40))
    await asyncio.sleep(1)
    await websocket.send(str(linear_vel) + "," + str(0))
    await asyncio.sleep(2)
    response = await websocket.recv()
    frame = decode.decode_frame(response)
    #current_state, linear_vel, angular_vel, img, filtered_img = processing.analyse_frame(frame, current_state, (0, 0), 0)
    images.append(frame)  # Add image to list
    #cv2.imshow("frame",frame)
    cv2.imwrite(f'images/image_{7}.jpg', frame)

    turns = 7



    print("Done")
    #stitcher=cv2.Stitcher.create(cv2.Stitcher_PANORAMA)
    #status,panorama=stitcher.stitch(frame)
    #cv2.imwrite('images/panorama.png',panorama)
async def run_test():
    async with websockets.connect(ip+'ws') as websocket:
        while True:
            await receive_camera_frame(websocket)
            if turns >= 6:  # If 6 turns completed, then break the loop
                break
asyncio.run(run_test())
