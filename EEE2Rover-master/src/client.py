import asyncio
# import aiohttp
import cv2
import websockets
import processing 
import decode 
import dfs
import random
import numpy as np

ip = "ws://192.168.4.1/"
async def test():
    async with websockets.connect(ip+'ws') as websocket:
        while True:
            # await websocket.send("hello")
            response = await websocket.recv()
            print(response)
            img = decode.decode_frame(response[:-11])
            x, y = decode.decode_position(response[-11:])
            walls, offset =processing.analyse_frame(img)
            await websocket.send(ip+"movement?offset="+offset)
            cv2.destroyAllWindows()    

async def run_test():
    while True:
        await test()
asyncio.run(run_test())


# ip = "http://192.168.4.1/"

# async def move_forward(session, value):
#     async with session.post(ip+'moveForward', data={'value': value}) as resp:
#         print(await resp.text())

# async def move_backward(session, value):
#     async with session.post(ip+'moveBackward', data={'value': value}) as resp:
#         print(await resp.text())

# async def run_test():
#     async with aiohttp.ClientSession() as session:
#         while True:
#             # get value to move forward or backward here, e.g. from your image analysis
#             value = get_value_from_image_analysis()
#             await move_forward(session, value)
#             # or await move_backward(session, value)

# asyncio.run(run_test()) send a moveforward(0,0) before each command