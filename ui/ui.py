"""Websockets example showing messages from connected clients and broadcasting via button click.

NOTE: NiceGUI already handles all the communication for you, so you don't need to worry about websockets and the like normally.
This example is only for advanced use cases where you want to allow other, non-NiceGUI clients to connect to your server.
"""
import asyncio
from typing import Set

import websockets
from websockets.server import WebSocketServerProtocol

from nicegui import app, ui
import pickle
import numpy as np
import time

CONNECTIONS: Set[WebSocketServerProtocol] = set()

dark = ui.dark_mode()
dark.enable()

def res(event):
    global I
    # print(img.source)
    # img.style(add="border-width: 5px")    
    

    # from PIL import Image
    # bk_img = np.random.randint(0, 255, (200, 200), dtype=np.uint8)
    # s = np.random.randint(240)
    # bk_img[s:s+10] = 1
    # image = Image.fromarray(bk_img)
    
    
    # image.save(f"./doc/ex_{I}.png")
    
    # import time
    # time.sleep(1)

    # img.source = f"./doc/ex_{I}.png"
    # I += 1
    # I = I%5


    websockets.broadcast(CONNECTIONS, 'See you!')

def get_image(data):
    # print(data)
    # with ui.column():
    #     ui.label(str(data))
    data = pickle.loads(data)
    if data[0] == 'img':
        # print('--')
        index = data[1]
        img.set_source(f"./tmp/ex.png")


def drag_tx(x, y):
    axis = 'TX'
    websockets.broadcast(CONNECTIONS, pickle.dumps([x, y, axis]) )

def drag_ty(x, y):
    axis = 'TY'
    websockets.broadcast(CONNECTIONS, pickle.dumps([x, y, axis]) )

def drag_tz(x, y):
    axis = 'TZ'
    websockets.broadcast(CONNECTIONS, pickle.dumps([x, y, axis]) )


def drag_rx(x, y):
    axis = 'RX'
    websockets.broadcast(CONNECTIONS, pickle.dumps([x, y, axis]) )

def drag_ry(x, y):
    axis = 'RY'
    websockets.broadcast(CONNECTIONS, pickle.dumps([x, y, axis]) )

def drag_rz(x, y):
    axis = 'RZ'
    websockets.broadcast(CONNECTIONS, pickle.dumps([x, y, axis]) )

drag_list = {
    'TX': drag_tx,
    'TY': drag_ty,
    'TZ': drag_tz,
    'RX': drag_rx,
    'RY': drag_ry,
    'RZ': drag_rz,
}


# with ui.row().style("min-width: 80%"):
with ui.column().classes('w-full'):
    
    # with ui.row():
    with ui.card().classes('text-center items-center').style(" width: 70%; height: 500px"):
        img = ui.interactive_image("./tmp/ex.png")
        ui.timer(interval=0.1, callback=lambda: img.set_source(f'/tmp/ex.png?{time.time()}'))

        # img = ui.image()
    with ui.row():
        with ui.card().classes('text-center items-center'):
            ui.label('Control').classes('text-2xl')

            with ui.row():
                with ui.column().classes("items-center"):                        
                    ui.joystick(color='blue', size=70, 
                            on_move=lambda e: drag_tx(float(e.y), float(e.x)),
                            on_end=lambda e: drag_tx(0, 0) )
                    ui.label('TX')
                with ui.column().classes("items-center"):                        
                    ui.joystick(color='blue', size=70, 
                            on_move=lambda e: drag_ty(float(e.y), float(e.x)),
                            on_end=lambda e: drag_ty(0, 0) )
                    ui.label('TY')
                with ui.column().classes("items-center"):                        
                    ui.joystick(color='blue', size=70, 
                            on_move=lambda e: drag_tz(float(e.y), float(e.x)),
                            on_end=lambda e: drag_tz(0, 0) )
                    ui.label('TZ')
                with ui.column().classes("items-center"):                        
                    ui.joystick(color='blue', size=70, 
                            on_move=lambda e: drag_rx(float(e.y), float(e.x)),
                            on_end=lambda e: drag_rx(0, 0) )
                    ui.label('RX')
                with ui.column().classes("items-center"):                        
                    ui.joystick(color='blue', size=70, 
                            on_move=lambda e: drag_ry(float(e.y), float(e.x)),
                            on_end=lambda e: drag_ry(0, 0) )
                    ui.label('RY')
                with ui.column().classes("items-center"):                        
                    ui.joystick(color='blue', size=70, 
                            on_move=lambda e: drag_rz(float(e.y), float(e.x)),
                            on_end=lambda e: drag_rz(0, 0) )
                    ui.label('RZ')
    
        with ui.card().classes('text-center items-center'):
            connections_label = ui.label('Control').classes('text-2xl')

            ui.button("Click for resonse", on_click=res)

            ui.separator().classes('mt-6')
            ui.label('incoming messages:')
            col = ui.column().classes('ml-4')


async def handle_connect(websocket: WebSocketServerProtocol):
    """Register the new websocket connection, handle incoming messages and remove the connection when it is closed."""
    try:
        CONNECTIONS.add(websocket)
        # connections_label.text = len(CONNECTIONS)
        while True:
            async for data in websocket:
                # get_image(data)
                # time.sleep(2)
                pass


    finally:
        CONNECTIONS.remove(websocket)
        # connections_label.text = len(CONNECTIONS)


async def start_websocket_server():
    async with websockets.serve(handle_connect, 'localhost', 8765):
        await asyncio.Future()

# start the websocket server when NiceGUI server starts
app.on_startup(start_websocket_server)
app.add_static_files("/doc", "./doc")
app.add_static_files("/tmp", "./tmp")
ui.run()