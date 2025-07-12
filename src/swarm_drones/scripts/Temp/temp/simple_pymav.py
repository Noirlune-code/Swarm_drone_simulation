from pymavlink import mavutil
import time

the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
the_connection.wait_heartbeat()
print("heartbeat receoved")




# the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
#                                 the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED, int(0b110111000111), 0,0,0, 1, 1, 0, 0, 0, 0, 0, 0))

# time_init = time.time()

# while time.time()- time_init < 3:
#     print(round(time_init-time.time(),2))
#     # the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
#     #                             the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED, int(0b110111000111), 0,0,0, 1, 1, 0, 0, 0, 0, 0, 0))
#     the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
#                 10, the_connection.target_system,
#                 the_connection.target_component,
#                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
#                 int(0b110111111000),
#                 int(-35.3632622 * 1e7), int(149.1652155 * 1e7), 1, 0, 0, -0, 0, 0, 0, 0, 0
#             ))
    
#     time.sleep(0.1)


# the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
#                                 the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED, int(0b110111000111), 0,0,0, 1, 0, 0, 0, 0, 0, 0, 0))


the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                10, the_connection.target_system,
                the_connection.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                int(0b100111111000),
                int(-35.3632622 * 1e7), int(149.1652595 * 1e7), 1, 0, 0, -0, 0, 0, 0, 0, 0
            ))
# Latitude: -35.3632622, Longitude: 149.1651935
time.sleep(0.1)
print("Command finished stop")
# while True:
#     the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
#                                 the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED, int(0b000111000111), 0,0,0, 0, 0, 0, 0, 0, 0, 0, 0))
#     time.sleep(0.1)

# print( -35.3632622 * 1e7, 149.1651935 * 1e7,)

print(int(-35.3632622 * 1e7), int(149.1652375 * 1e7))