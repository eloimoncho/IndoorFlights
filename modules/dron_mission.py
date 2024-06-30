import json
import math
import threading
import time
from pymavlink import mavutil
from modules.dron_goto import _goto, goto
from modules.dron_RTL_Land import _goDown
from modules.dron_arm import _arm
from modules.dron_takeOff import  _takeOff


def _executeMission (self, mission, callback=None, params = None):
    '''La mision debe especificarse en json, de acuerdo con el formato de este ejemplo:
        {
            "takeOffAlt": 5,
            "waypoints":
                [
                    {
                        'lat': 41.2763410,
                        'lon': 1.9888285,
                        'alt': 12
                    },
                    {
                        'lat': 41.27623,
                        'lon': 1.987,
                        'alt': 14
                    }
                ]

        }
        El dron armará, despegara hasta la altura indicada, navegará por los waypoints y acabará
        con un RTL
        '''



    takOffAlt = mission['takeOffAlt']

    waypoints = mission['waypoints']


    # preparamos la misión para cargarla en el dron

    wploader = []
    seq = 0  # Waypoint sequence begins at 0
    # El primer wp debe ser la home position.
    # Averiguamos la home position
    self.vehicle.mav.command_long_send(
        self.vehicle.target_system,
        self.vehicle.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0)

    msg = self.vehicle.recv_match(type='HOME_POSITION', blocking=True)
    msg = msg.to_dict()
    lat = msg['latitude']
    lon = msg['longitude']
    alt = msg['altitude']


    # añadimos este primer waypoint a la mision
    wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
        0, 0, seq, 0, 16, 0, 0, 0, 0, 0, 0,
        lat,
        lon,
        alt
    ))
    seq = 1

    # El siguiente elemento de la mision debe ser el comando de takeOff, en el que debemos indicar una posición
    # que será también la home position

    wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
        self.vehicle.target_system, self.vehicle.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, True,
        0, 0, 0, 0,
        lat, alt, takOffAlt
    ))
    seq = 2

    # Ahora añadimos los waypoints de la ruta
    for wp in waypoints:
        wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.vehicle.target_system, self.vehicle.target_component, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, True,
            0, 0, 0, 0,
            int(wp["lat"] * 10 ** 7), int(wp["lon"] * 10 ** 7), int(wp["alt"])
        ))
        seq += 1  # Increase waypoint sequence for the next waypoint

    # añadimos para acabar un RTL
    wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
        self.vehicle.target_system, self.vehicle.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, True,
        0, 0, 0, 0,
        0, 0, 0
    ))
    # borramos la misión que tiene ahora el autopiloto
    self.vehicle.mav.mission_clear_all_send( self.vehicle.target_system,  self.vehicle.target_component)

    ack = self.vehicle.recv_match(type='MISSION_ACK', blocking=True)

    # Enviamos el numero de items de la nueva misión
    self.vehicle.waypoint_count_send(len(wploader))

    # Enviamos los items

    for i in range(len(wploader)):

        msg = self.vehicle.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True)

        print(f'Sending waypoint {msg.seq}/{len(wploader) - 1}')
        self.vehicle.mav.send(wploader[msg.seq])

        if msg.seq == len(wploader) - 1:
            break


    ack = self.vehicle.recv_match(type='MISSION_ACK', blocking=True)

    # armamos y damos la orden de ejecutar la misión

    mode = 'GUIDED'
    # Get mode ID
    mode_id = self.vehicle.mode_mapping()[mode]
    self.vehicle.mav.set_mode_send(
        self.vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    arm_msg = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True)

    self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    self.vehicle.motors_armed_wait()

    self.vehicle.mav.command_long_send(
        self.vehicle.target_system,
        self.vehicle.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0)
    self.state = 'flying'
    # esperamos a que acabe la mision
    time.sleep(10)
    while True:
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout = 3)
        if msg:
            msg = msg.to_dict()
            alt = float(msg['relative_alt'] / 1000)
            print(alt)
            if alt < 0.5:
                break
        time.sleep(1)
    self.state = 'connected'
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




def executeMission(self,flightPlan, blocking=True, callback=None, params = None):
    if blocking:
        self._executeMission(flightPlan)
    else:
        missionThread = threading.Thread(target=self._executeMission, args=[flightPlan, callback, params])
        missionThread.start()