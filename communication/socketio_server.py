import asyncio
import websockets
import socketio
import eventlet
from trajectory import *

HOST = '192.168.0.114'
PORT = 8765

async def execute_trajectory(websocket, path):
    data = await websocket.recv()
    print(f"Received: {data}")
    
    state, state_type, state_value = data.split(',')

    if state.strip().upper() == 'ACTION' and state_type.strip().upper() == 'TRAJECTORY':
        trajectory = Trajectory(state_value.strip())
        trajectory.start()
    else:
        print("ERROr: Unknown command")

sio = socketio.Server()
app = socketio.WSGIApp(sio, static_files={
    '/': {'content_type': 'text/html', 'filename': 'index.html'}
})

@sio.event
def connect(sid, environ):
    print('connect ', sid)
    return "CONNECTED"

@sio.event
def my_message(sid, data):
    print('message ', data)

@sio.event
def disconnect(sid):
    print('disconnect ', sid)

if __name__ == '__main__':
    eventlet.wsgi.server(eventlet.listen((HOST, PORT)),app)
