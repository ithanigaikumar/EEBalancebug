import asyncio
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
            img = decode.decode_frame(response)
            walls, offset =processing.analyse_frame(img)
            await websocket.send(ip+"movement?offset="+offset)
            cv2.destroyAllWindows()    

async def run_test():
    while True:
        await test()
asyncio.run(run_test())