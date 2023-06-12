import asyncio
import cv2
import websockets
import processing 
import decode 
async def test():
    async with websockets.connect('ws://192.168.4.1/ws') as websocket:
        while True:
            # await websocket.send("hello")
            response = await websocket.recv()
            print(response)
            img = decode.decode_chunks(response)
            processing.analyseFrame(img)
           

async def run_test():
    while True:
        await test()

asyncio.run(run_test())