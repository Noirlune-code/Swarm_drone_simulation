import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from pymavlink import mavutil
import time

class swarm_drone():

    def __init__(self, instance, ros_node):
        self.instance = instance
        port = 14550 + 10 * self.instance
        address = 'udp:127.0.0.1:' + str(port)
        self.master = mavutil.mavlink_connection(address)
        self.master.wait_heartbeat()
        self.node = ros_node
        self.node.get_logger().info(f"Heartbeat from system instance for swarm drone {self.instance}")

    def set_guided_mode(self):
        current_mode = self.master.flightmode
        if current_mode == 'GUIDED':
            self.node.get_logger().info(f"Drone already in GUIDED mode {self.instance}")
            return
        self.master.set_mode_apm('GUIDED')
        self.node.get_logger().info(f"Switched to GUIDED mode for swarm drone {self.instance}")
        
    def check_guided_mode(self):
        current_mode = self.master.flightmode
        if current_mode == 'GUIDED':
            return True
        else:
            return False

    def arm_throttle(self):
        self.master.recv_match(type='HEARTBEAT', blocking=True)
        if self.master.motors_armed():
            self.node.get_logger().info(f"Drone is already armed {self.instance}")
            return
        self.master.arducopter_arm()
        self.node.get_logger().info(f"Sent arming command for swarm drone {self.instance}")
        self.master.motors_armed_wait()
        self.node.get_logger().info(f"Drone is armed for swarm drone {self.instance}")

    def check_arm_throttle(self):
        self.master.recv_match(type='HEARTBEAT', blocking=True)
        if self.master.motors_armed():
            return True
        else:
            return False

    def takeoff_to_altitude(self, altitude = 1.0):
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        current_alt = msg.relative_alt/1000.0
        if current_alt != 0:
            self.node.get_logger().info(f"Already in air {self.instance}")
            return True
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,         # confirmation
        0, 0, 0,    # param 1-3: not used
        float('nan'), 0, 0, altitude  # yaw, lat, lon, alt
    )
        self.node.get_logger().info(f"Takeoff command to {altitude}m sent for swarm drone {self.instance}")
    
    def check_takeoff(self):
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        current_alt = msg.relative_alt/1000.0
        if current_alt > 0.1:
            return True
        else:
            return False

    def land(self):
        current_mode = self.master.flightmode
        if current_mode == 'LAND':
            self.node.get_logger().info(f"Already in LAND mode {self.instance}")
            return True
        self.master.set_mode_apm('LAND')
        self.node.get_logger().info(f"Landing initiated.. for swarm drone {self.instance}")

    def check_land(self):
        current_mode = self.master.flightmode
        if current_mode == 'LAND':
            return True
        else:
            return False

    def send_vel_command(self, x , y , z , yaw):
        # positive yaw for right turn
        if x != 0 or y !=0 or z != 0 or yaw != 0:
            self.master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.master.target_system,
                        self.master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b010111000111), 0, 0, 0, y,x, -z, 0, 0, 0, 0, -yaw * 0.5))
            self.node.get_logger().info(f"Velocity command send {x} {y}, {z},{-yaw*0.5} for drone {self.instance}")
    

class JoystickListener(Node):
    def __init__(self):
        super().__init__('joystick_pymav')

        # Initialize joystick values
        self.x = self.y = 0.0
        self.up = self.down = 0.0
        self.left = self.right = 0.0
        self.guided = self.arm = self.takeoff = self.land = 0.0

        self.swarm_drones = [swarm_drone(0,self),swarm_drone(1,self),swarm_drone(2, self),swarm_drone(3,self), swarm_drone(4,self)]

        # Subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'controller_data',
            self.joystick_callback,
            10
        )

        # Timer loop: 50 ms interval = 20 Hz
        self.timer = self.create_timer(0.5, self.mode_control_loop)
        self.timer = self.create_timer(0.05,self.vel_control_loop)

    def joystick_callback(self, msg):
        data = msg.data
        if len(data) != 8:
            self.get_logger().warn("Incorrect joystick message length")
            return

        (
        self.x, self.y,
        self.height, self.yaw,
        self.guided, self.arm, self.takeoff, self.land,
        ) = data

    def mode_control_loop(self):
        # Run this every 50 ms
        if self.guided == 1.0:
            for items in self.swarm_drones:
                items.set_guided_mode()

        if self.arm == 1.0:
            for items in self.swarm_drones:
                self.get_logger().warn(str(items.check_guided_mode()))
                if items.check_guided_mode():
                    items.arm_throttle()
                else:
                    self.get_logger().warn("Set to GUIDED mode before throttle")
        if self.takeoff == 1.0:
            for items in self.swarm_drones:
                if items.check_arm_throttle():
                    items.takeoff_to_altitude()
                else:
                    self.get_logger().warn("Arm throttle not done")

        if self.land == 1.0:
            for items in self.swarm_drones:
                items.land()

        # for items in self.swarm_drones:
        #     if items.check_takeoff():
        #         items.send_vel_command(self.x, self.y, self.height, self.yaw)
    
    def vel_control_loop(self):
        for items in self.swarm_drones:
            if items.check_takeoff():
                items.send_vel_command(self.x,self.y, self.height, self.yaw)




def main():
    rclpy.init()
    node = JoystickListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
