import asyncio
import websockets
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

start_server = websockets.serve(execute_trajectory, HOST, PORT)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
