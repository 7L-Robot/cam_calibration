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

CONNECTIONS: Set[WebSocketServerProtocol] = set()

def res(event):
    websockets.broadcast(CONNECTIONS, 'See you!')
    img.source = "./doc/in.gif"
    # img.style(add="border-width: 5px")    

def drag(x, y, axis):
    if axis is not None:
        websockets.broadcast(CONNECTIONS, pickle.dumps([x, y, axis]) )


# with ui.row().style("min-width: 80%"):
with ui.column().classes('w-full'):
    
    # with ui.row():
    with ui.card().classes('text-center items-center').style(" width: 70%; height: 500px"):
        img = ui.image("./doc/ex.gif")
        # img = ui.image()
        # print(img)
    
    with ui.row():
        with ui.card().classes('text-center items-center'):
            ui.label('Control').classes('text-2xl')

            with ui.row():    
                with ui.column().classes("items-center"):
                    x_control = ui.joystick(color='blue', size=70, 
                            on_move=lambda e: drag(float(e.y), float(e.x), 'x'),
                            on_end=lambda e: drag(0, 0, None) )
                    ui.label("X")
                with ui.column().classes("items-center"):
                    y_control = ui.joystick(color='blue', size=70, 
                            on_move=lambda e: drag(float(e.y), float(e.x), 'y'),
                            on_end=lambda e: drag(0, 0, None) )
                    ui.label("Y")
                with ui.column().classes("items-center"):
                    z_control = ui.joystick(color='blue', size=70, 
                            on_move=lambda e: drag(float(e.y), float(e.x), 'z'),
                            on_end=lambda e: drag(0, 0, None) )
                    ui.label("Z")

            ui.button("Click for resonse", on_click=res)
            ui.separator().classes('mt-6')
            ui.label('incoming messages:')
            messages = ui.column().classes('ml-4')
    
        with ui.card().classes('text-center items-center'):
            ui.label('Control').classes('text-2xl')

            ui.button("Click for resonse", on_click=res)
            ui.separator().classes('mt-6')
            ui.label('incoming messages:')





async def handle_connect(websocket: WebSocketServerProtocol):
    """Register the new websocket connection, handle incoming messages and remove the connection when it is closed."""
    try:
        CONNECTIONS.add(websocket)
        # connections_label.text = len(CONNECTIONS)
        async for data in websocket:
            with messages:
                ui.label(str(data))
    finally:
        CONNECTIONS.remove(websocket)
        # connections_label.text = len(CONNECTIONS)


async def start_websocket_server():
    async with websockets.serve(handle_connect, 'localhost', 8765):
        await asyncio.Future()

# start the websocket server when NiceGUI server starts
app.on_startup(start_websocket_server)

ui.run()