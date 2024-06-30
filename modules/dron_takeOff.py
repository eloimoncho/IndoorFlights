import threading
import time
from pymavlink import mavutil

def request_message_interval(self, message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    self.vehicle.mav.command_long_send(
        self.vehicle.target_system, self.vehicle.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

def _takeOff(self, aTargetAltitude,callback=None, params = None):
    self.state = "takingOff"
    self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, aTargetAltitude)
    #msg = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)

    #ahora tengo que preguntar por la altura para detectar cuándo ha alcanzado la altura indicada
    '''  self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 2)
    while True:
        print ('espero altura')
        try:
            print('****', self.vehicle.recv_match().to_dict())
        except:
            print ('break')
            break
        time.sleep(0.1)'''
    print ('empezamos a mirar')
    while True:
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        #print('meg ', msg)
        if msg:
            msg = msg.to_dict()
            alt = float(msg['relative_alt'] / 1000)
            print ('alt ', alt)
            if alt >= aTargetAltitude * 0.90:
                print("Reached target altitude")
                break
            time.sleep(1)


    self.state = "flying"
    if callback != None:
        if self.id == None:
            if params == None:
                callback()
            else:
                callback(params)
        else:
            if params == None:
                callback(self.id)
            else:
                callback(self.id, params)



def takeOff(self, aTargetAltitude, blocking=True, callback=None, params = None):
    if self.state == 'armed':
        if blocking:
            self._takeOff(aTargetAltitude)
        else:
            takeOffThread = threading.Thread(target=self._takeOff, args=[aTargetAltitude, callback, params])
            takeOffThread.start()
        return True
    else:
        return False
