import pygame
import math
from pymavlink import mavutil



# === CONFIGURATION === #
CELL_SIZE = 80
GAP = 10
FONT_SIZE = 24
BUTTON_HEIGHT = 40
MAX_SELECTIONS = 5
AUTO_PLAY_INTERVAL_MS = 500

WHITE = (255, 255, 255)
GRAY = (210, 210, 210)
BLACK = (0, 0, 0)
YELLOW = (255, 255, 100)
BLUE = (50, 100, 255)
GREEN = (50, 200, 50)
RED = (200, 50, 50)
ORANGE = (255, 165, 0)
# FONT = pygame.font.SysFont(None, FONT_SIZE)
pygame.init()
FONT = pygame.font.SysFont(None, FONT_SIZE)



class logger():
    def info(string_dat):
        print(string_dat)

# === GRID SELECTOR === #
class GridSelector:
    def __init__(self, grid_size, x_offset, y_offset, max_selections=5):
        self.grid_size = grid_size
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.max_selections = max_selections
        self.selected = []
        self.locked = False

    def draw(self, surface):
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                x = self.x_offset + GAP + col * (CELL_SIZE + GAP)
                y = self.y_offset + GAP + row * (CELL_SIZE + GAP)
                rect = pygame.Rect(x, y, CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(surface, GRAY, rect)

                if (col, row) in self.selected:
                    pygame.draw.rect(surface, YELLOW, rect.inflate(-10, -10))

                pygame.draw.rect(surface, BLACK, rect, 2)

    def handle_click(self, mouse_pos):
        if self.locked:
            return

        mx, my = mouse_pos
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                x = self.x_offset + GAP + col * (CELL_SIZE + GAP)
                y = self.y_offset + GAP + row * (CELL_SIZE + GAP)
                rect = pygame.Rect(x, y, CELL_SIZE, CELL_SIZE)
                if rect.collidepoint(mx, my):
                    pos = (col, row)
                    if pos in self.selected:
                        self.selected.remove(pos)
                    elif len(self.selected) < self.max_selections:
                        self.selected.append(pos)
                    return

    def reset(self):
        self.selected = []
        self.locked = False

    def lock(self):
        self.locked = True

    def get_selected(self):
        return list(self.selected)


# === COMMAND GENERATOR === #
class DroneAssignmentEngine:
    def __init__(self, grid_size):
        self.grid_size = grid_size
        self.drone_start_positions = {
            "D1": (0, 0),
            "D2": (0, 1),
            "D3": (0, 2),
            "D4": (0, 3),
            "D5": (0, 4),
        }

    def generate_commands(self, selected_cells):
        commands = []
        drone_available = set(self.drone_start_positions.keys())

        # Group by y row (from bottom to top)
        selected_by_row = {}
        for x, y in selected_cells:
            selected_by_row.setdefault(y, []).append((x, y))

        for y in sorted(selected_by_row.keys(), reverse=True):
            for x, _ in sorted(selected_by_row[y]):
                assigned = False
                for drone_id in sorted(self.drone_start_positions):
                    if (
                        self.drone_start_positions[drone_id] == (0, x)
                        and drone_id in drone_available
                    ):
                        commands.append((drone_id, x, y))
                        drone_available.remove(drone_id)
                        assigned = True
                        break

                if not assigned:
                    nearest_drone = None
                    nearest_distance = float("inf")
                    for drone_id in drone_available:
                        dx = self.drone_start_positions[drone_id][1]
                        dist = abs(dx - x)
                        if dist < nearest_distance:
                            nearest_distance = dist
                            nearest_drone = drone_id

                    if nearest_drone:
                        drone_available.remove(nearest_drone)
                        commands.append((nearest_drone, x, 0))
                        commands.append((nearest_drone, x, y))
        return commands


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

Drone0 = swarm_drone(0)
Drone1 = swarm_drone(1)
Drone2 = swarm_drone(2)
Drone3 = swarm_drone(3)
Drone4 = swarm_drone(4)

Drone_list = [Drone0, Drone1, Drone2, Drone3, Drone4]

location0 = Drone0.get_lat_lon()
location1 = Drone1.get_lat_lon()
location2 = Drone2.get_lat_lon()
location3 = Drone3.get_lat_lon()
location4 = Drone4.get_lat_lon()

origin = location0
print(Drone0.latlon_to_meters(location0,location1))

print(Drone0.meters_to_latlon_offset(location0, (2,0)))
# print(Drone0.latlon_to_meters(location0[0],location0[1], location1[0], location1[1]))

# origin_location = Drone0.get_lat_lon()

cmds = [(3, 2, 4),
(2, 2, 0),
(2, 2, 3),
(1, 1, 0),
(1, 1, 2),
(4, 3, 2),
(5, 2, 0),
(5, 2, 1)]

# cmds = [(0,(5,0))]
for command in cmds:
    print()
    target_location = Drone0.meters_to_latlon_offset(location0, (-command[2], command[1]))
    Drone_list[command[0]].set_position_target_global_int(target_location)
    var = input()
        
# target_location = Drone0.meters_to_latlon_offset(location0, cmds[0][1])
# print(location0, target_location)
# Drone_list[cmds[0][0]].set_position_target_global_int(target_location)
    

# for swarm in Drone_list:
#     swarm.set_guided_mode()

# var = input("Pass")

# for swarm in Drone_list:
#     swarm.arm_throttle()
#     swarm.takeoff_to_altitude()
#     var = input("pass")


