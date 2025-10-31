#!/usr/bin/env python3
"""
inject_obstacle_distance_sensor.py

向 SITL 飞控注入 DISTANCE_SENSOR 消息，模拟前方障碍物。
兼容 ArduPilot SITL ardupilotmega MAVLink 方言。
QGC Proximity 界面会显示障碍物。
"""

import time
from pymavlink import mavutil

CONN_STR = "udp:127.0.0.1:14550"

MIN_DISTANCE_CM = 20
MAX_DISTANCE_CM = 400
OBSTACLE_DISTANCE_CM = 150
SEND_HZ = 5

def main():
    print("Connecting to MAVLink on", CONN_STR)
    master = mavutil.mavlink_connection(CONN_STR, autoreconnect=True)
    print("Waiting for heartbeat from system...")
    master.wait_heartbeat(timeout=10)
    print("Heartbeat received. Start sending DISTANCE_SENSOR at %.1f Hz" % SEND_HZ)

    send_interval = 1.0 / SEND_HZ

    try:
        while True:
            # uint32 毫秒时间戳
            time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF

            master.mav.distance_sensor_send(
                time_boot_ms,
                MIN_DISTANCE_CM,
                MAX_DISTANCE_CM,
                OBSTACLE_DISTANCE_CM,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                0,  # id
                0,  # orientation FRONT
                0   # covariance
            )

            time.sleep(send_interval)

    except KeyboardInterrupt:
        print("Stopped by user")

if __name__ == "__main__":
    main()
