import json
import threading

import pymavlink
from pymavlink.mavutil import default_native
import pymavlink.dialects.v20.all as dialect
from pymavlink import mavutil
import time

def _setGEOFence(self, fence_waypoints, callback=None, params = None):
    fence_list = json.loads(fence_waypoints)
    print ('puntos :', fence_list)
    #indico el número de waypints del fence
    FENCE_TOTAL = "FENCE_TOTAL".encode(encoding="utf-8")
    #El primero se envía tres veces (una como FENCE_RETURN_POINT, otra como primer punto del fence y otra como
    # último punto que cierra el fence
    fence_points = len(fence_list)+2
    while True:
        message = dialect.MAVLink_param_set_message(target_system=self.vehicle.target_system,
                                                    target_component=self.vehicle.target_component,
                                                    param_id=FENCE_TOTAL, param_value=fence_points,
                                                    param_type=dialect.MAV_PARAM_TYPE_REAL32)
        self.vehicle.mav.send(message)
        message = self.vehicle.recv_match(type="PARAM_VALUE", blocking=True)
        message = message.to_dict()

        if message["param_id"] == "FENCE_TOTAL":
            if int(message["param_value"]) == fence_points:
                break
    print ('ya envie el numero de puntos')
    # el primer wp lo envio primero para que haga de FENCE_RETURN_POINT
    message = dialect.MAVLink_fence_point_message(target_system=self.vehicle.target_system,
                                                  target_component=self.vehicle.target_component,
                                                  idx=0, count=fence_points, lat=fence_list[0]['lat'],
                                                  lng=fence_list[0]['lon'])
    self.vehicle.mav.send(message)
    idx = 0
    while idx < len(fence_list):
        message = dialect.MAVLink_fence_point_message(target_system=self.vehicle.target_system,
                                                      target_component=self.vehicle.target_component,
                                                      idx=idx, count=fence_points, lat=fence_list[idx]['lat'],
                                                      lng=fence_list[idx]['lon'])
        self.vehicle.mav.send(message)
        idx = idx + 1

    # envio de nuevo el primero para que cierra el poligono
    message = dialect.MAVLink_fence_point_message(target_system=self.vehicle.target_system,
                                                      target_component=self.vehicle.target_component,
                                                      idx=idx, count=fence_points, lat=fence_list[0]['lat'],
                                                      lng=fence_list[0]['lon'])
    self.vehicle.mav.send(message)

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


def setGEOFence(self, fence_waypoints, blocking=True, callback=None, params = None):
    if blocking:
        self._setGEOFence(fence_waypoints)
    else:
        setGEOFenceThread= threading.Thread(target=self._setGEOFence, args=[fence_waypoints, callback, params])
        setGEOFenceThread.start()