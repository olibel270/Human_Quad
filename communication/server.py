import asyncio
import websockets
from trajectory import *
import zerorpc
import rospy
from sensor_msgs.msg import BatteryState
import sys
sys.path.insert(0, '../positioning/')
from positioning_pub import PositionPub

HOST = '192.168.0.114'
PORT = 8765

class RPC(object):
    def __init__(self, hedge):
        self.hedge = hedge
        self.position_link_status = 1 if self.hedge.positionUpdated else 0
        self.timer = threading.Timer(3, self.position_link_check) 
        self.timer.start()
    

    def reset_timer(self):
        self.timer = threading.Timer(3, self.position_link_check)

    def position_link_check(self):
        if self.hedge.positionUpdated:
            self.position_link_status = 0
            self.hedge.positionUpdated = 0
            self.timer.cancel()
            self.reset_timer()
            self.timer.start()
        else:
            self.position_link_status = 0
            self.timer.cancel()
            self.reset_timer()
            self.timer.start()



    def execute_trajectory(self, data):
        print(f"Received: {data}")
        
        state, state_type, state_value = data.split(',')
    
        if state.strip().upper() == 'ACTION' and state_type.strip().upper() == 'TRAJECTORY':
            trajectory = Trajectory(state_value.strip())
            trajectory.start()
        else:
            print("ERROR: Unknown command")
            return None

        return "Trajectory executed"

    @zerorpc.stream
    def get_battery_level(self):
        rospy.init_node('battery_listener')
        battery_state_msg = rospy.wait_for_message('/mavros/battery', BatteryState)
        battery_level = battery_state_msg.percentage
        battery_level = 0
        while True:
            yield battery_level
            battery_level += 1
            if battery_level == 500:
                return battery_level

    @zerorpc.stream
    def get_position_link(self):
        state, state_type, state_value = data.split(',')

        if state.strip().upper() == 'STATUS' and state_type.strip().upper() == 'POSITION':
            while True:
                yield self.position_link_status
                self.position_link_status = 1 if self.hedge.positionUpdated else 0
                if not self.position_link_status:
                    break

if __name__ == "__main__":
    position_pub = PositionPub()
    position_pub.positioning_pub()
    hedge = position_pub.hedge
    s = zerorpc.Server(RPC(hedge))
    s.bind("tcp://0.0.0.0:4242")
    print("RPC Server started on port 4242...")
    s.run()
