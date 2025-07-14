from pymavlink import mavutil
import math



class logger():
    def info(string_dat):
        print(string_dat)



class swarm_drone:

    def __init__(self, instance):
        self.instance = instance
        port = 14550 + 10 * self.instance
        address = 'udp:127.0.0.1:' + str(port)
        self.master = mavutil.mavlink_connection(address)
        self.master.wait_heartbeat()
        logger.info(f"Heartbeat from system instance for swarm drone {self.instance}")
        self.postion_global = (0.0, 0.0)

    def set_guided_mode(self):
        current_mode = self.master.flightmode
        if current_mode == 'GUIDED':
            logger.info(f"Drone already in GUIDED mode {self.instance}")
            return
        self.master.set_mode_apm('GUIDED')
        logger.info(f"Switched to GUIDED mode for swarm drone {self.instance}")
        
    def check_guided_mode(self):
        current_mode = self.master.flightmode
        return current_mode == 'GUIDED'

    def arm_throttle(self):
        self.master.recv_match(type='HEARTBEAT', blocking=True)
        if self.master.motors_armed():
            logger.info(f"Drone is already armed {self.instance}")
            return
        self.master.arducopter_arm()
        logger.info(f"Sent arming command for swarm drone {self.instance}")
        self.master.motors_armed_wait()
        logger.info(f"Drone is armed for swarm drone {self.instance}")

    def check_arm_throttle(self):
        self.master.recv_match(type='HEARTBEAT', blocking=True)
        return self.master.motors_armed()

    def takeoff_to_altitude(self, altitude=1.0):
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        current_alt = msg.relative_alt / 1000.0
        if current_alt != 0:
            logger.info(f"Already in air {self.instance}")
            return True
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0,
            float('nan'), 0, 0, altitude
        )
        logger.info(f"Takeoff command to {altitude}m sent for swarm drone {self.instance}")

    def check_takeoff(self):
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        current_alt = msg.relative_alt / 1000.0
        return current_alt > 0.1

    def land(self):
        current_mode = self.master.flightmode
        if current_mode == 'LAND':
            logger.info(f"Already in LAND mode {self.instance}")
            return True
        self.master.set_mode_apm('LAND')
        logger.info(f"Landing initiated.. for swarm drone {self.instance}")

    def check_land(self):
        current_mode = self.master.flightmode
        return current_mode == 'LAND'

    def send_vel_command(self, x, y, z, yaw):
        if x != 0 or y != 0 or z != 0 or yaw != 0:
            self.master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10, self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                int(0b010111000111),
                0, 0, 0, y, x, -z, 0, 0, 0, 0, -yaw * 0.5
            ))
            logger.info(f"Velocity command sent: x={x}, y={y}, z={z}, yaw={-yaw*0.5} for drone {self.instance}")

    def send_position_command(self, location):
            self.master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10, self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                int(0b110111111000),
                0, 0, 0, 0, 0, -0, 0, 0, 0, 0, location
            ))


    @staticmethod
    def meters_to_latlon_offset(locat1, change_in_coord):
        
        dlat = change_in_coord[0] / 111320
        dlon = change_in_coord[1] / (40075000 * math.cos(math.radians(locat1[0])) / 360)
        return (locat1[0] + dlat, locat1[1] + dlon)

    @staticmethod
    def latlon_to_meters(locat1, locat2):
        dlat = locat2[0] - locat1[0]
        dlon = locat2[1] - locat1[1]
        avg_lat = (locat2[0] + locat1[0]) / 2.0
        dy = dlat * 111320
        dx = dlon * (40075000 * math.cos(math.radians(avg_lat)) / 360)
        return (dy, dx)

    def get_lat_lon(self):
        msg = None
        while msg is None:
            try:
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            except Exception as e:
                logger.error(f"Error receiving GPS data: {e}")
                return
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        logger.info(f"Latitude: {lat:.7f}, Longitude: {lon:.7f}")
        return (lat,lon)

    def set_position_target_global_int(self, locat):
    
        # # Convert lat/lon to integers
        # lat_int = int(locat[0] * 1e7)
        # lon_int = int(locat[1] * 1e7)

        # Type mask for using only position (ignore vel, acc, yaw, yaw_rate)
        type_mask = 0b110111111100  # = 3576 → only position used

        # Coordinate frame: MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
        coordinate_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

        self.master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                10, self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                int(0b100111111000),
                int(locat[0] * 1e7), int(locat[1] * 1e7), 1, 0, 0, -0, 0, 0, 0, 0, 0
            ))

        print(f"Sent target position: lat={locat[0]}, lon={locat[1]}")

    def check_position(self, target_location):
        current_location = self.get_lat_lon()
        diff = self.latlon_to_meters(target_location, current_location)
        if diff[0]**2 + diff[1]**2 <= 0.5:
            return True
        return False



class DroneAssignmentEngine:
    def __init__(self, initial_formation):
        self.drone_avail = initial_formation
    
    def nearest_avail(self, index):
        j = 0
        while True:
            p1 = index-j
            p2 = index + j
            if p2 < len(self.drone_avail) and self.drone_avail[p2] == 1:
                return p2
            elif p1 >= 0 and self.drone_avail[p1] == 1:
                return p1
            elif p1 < 0 and p2 >= len(self.drone_avail):
                return
            j = j + 1

    def generate_command(self, required_position):
        command_list = []
        drone_to_send = self.nearest_avail(required_position[0])
        if drone_to_send == required_position[0]:
                command_list.append([drone_to_send,required_position])
                self.drone_avail[drone_to_send] = 0
        else:
                command_list.append([drone_to_send, [required_position[0],0]])
                command_list.append([drone_to_send, [required_position[0], required_position[1]]])
                self.drone_avail[drone_to_send] = 0
        return command_list

    def generate_command_list(self, required_position_list,size):
    
        command_list = []
        j = size
        print(required_position_list)
        while j>= 0:
            i = size
            while i >=0:
                print(i,j)
                if (i,j) in required_position_list:
                    cmds = self.generate_command((i,j))
                    for cmd in cmds:
                        command_list.append(cmd)
                i = i -1
            j = j - 1
        return command_list

class x_y_to_lat_long:
    def __init__(self, start_location, end_location,size):
        self.x00_y00 = start_location
        self.xn0_yn0 = end_location
        self.x0n_y0n = (self.x00_y00[0] - self.x00_y00[1]- self.xn0_yn0[1], self.x00_y00[1]- self.xn0_yn0[0] + self.x00_y00[0])
        self.xnn_ynn = (self.xn0_yn0[0] - self.x00_y00[1]- self.xn0_yn0[1], self.xn0_yn0[1]- self.xn0_yn0[0] + self.x00_y00[0])
        self.n = size - 1
    def divide_by_location_t(self,locat1,locat2,t_ratio):
        x = locat1[0] + t_ratio * (locat2[0]-locat1[0])
        y = locat1[1] + t_ratio * (locat2[1]-locat1[1])
        return (x,y)
        
    def get_lat_lon(self, pos):
        ratio_x = pos[0]/self.n
        ratio_y = pos[1]/self.n
        pt1 = self.divide_by_location_t(self.x00_y00, self.xn0_yn0,ratio_x)
        pt2 = self.divide_by_location_t(self.x0n_y0n,self.xnn_ynn, ratio_x)
        
        return self.divide_by_location_t(pt1,pt2,ratio_y)
    def print_corners(self):
        print(self.x00_y00,self.xn0_yn0,self.x0n_y0n,self.xnn_ynn)
engine = DroneAssignmentEngine([1,1,1,1,1])


x_y_1 = x_y_to_lat_long((20,30),(60,30),5)

x_y_1.print_corners()

# print(engine.nearest_avail(1))

# print(engine.generate_command_list( [(1, 2), (2, 2), (3, 2), (2, 1), (2, 3)],5))


# aa = [(1, 2), (2, 2), (3, 2), (2, 1), (2, 3)]


# if (1,2) in aa :
#     print(True)