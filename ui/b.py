import asyncio
import websockets
import time
import pickle

async def ws_client(url):
    while True:
        async with websockets.connect(url) as websocket:
            # await websocket.send("Hello, I am PyPy.")
            response = await websocket.recv()
            res = pickle.loads(response)
            
        print(res)
        # time.sleep(1)

async def bye(url):
    for _ in range(5):
        async with websockets.connect(url) as websocket:
            await websocket.send("Bye.")
        time.sleep(1)



if __name__ == '__main__':
    # ws_client('ws://localhost:8765')
    asyncio.run(ws_client('ws://localhost:8765'))
    # print('here')
    # asyncio.run(bye('ws://localhost:8765'))
