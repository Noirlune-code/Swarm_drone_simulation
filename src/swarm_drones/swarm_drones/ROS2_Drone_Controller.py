"""
ControllerDroneUI – Full Modular Drone Control Interface using Joystick, DPad, Buttons, TextLabels, SpinBoxes

This is the main user interface class that integrates all modular widgets and logic into a single UI,
replacing the original monolithic code. This version cleanly uses modularized widgets for:
- Joystick with configurable max range (via SpinBox)
- DPad with customizable value strength (via SpinBox)
- Buttons for Guided, Arm, Takeoff, Land
- TextLabels for displaying velocity and status
- A dedicated "Apply" Button to commit SpinBox settings

This class handles all widget layout, updates, and ROS publishing logic.

────────────────────────────────────────────────────────────
FEATURES:
────────────────────────────────────────────────────────────
- All components styled and positioned for clarity
- Live velocity updates shown via labels
- Reusable components (DPad, Joystick, Button, SpinBox, TextLabel)
- Apply button updates Joystick/DPad sensitivity

To use, run the script and interact via mouse.

────────────────────────────────────────────────────────────
"""

import pygame
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from swarm_drones.Custom_python_widgets import Button, DPad, Joystick, SpinBox, TextLabel

class ControllerDrone(Node):
    def __init__(self):
        super().__init__('controller_drone')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'controller_data', 10)

        pygame.init()
        self.screen = pygame.display.set_mode((850, 520))
        pygame.display.set_caption("Modular Drone Controller UI")
        self.clock = pygame.time.Clock()
        self.running = True

        self.font = pygame.font.SysFont("arial", 24)
        self.big_font = pygame.font.SysFont("arial", 30, bold=True)

        # SpinBoxes and Apply button
        self.spinboxes = {
            "Joystick Max": SpinBox(position=(260, 80), width=120, height=40, value=1, step=1, min_value=1, max_value=500),
            "DPad Value": SpinBox(position=(610, 80), width=120, height=40, value=1, step=1, min_value=1, max_value=10)
        }
        self.apply_button = Button(position=(435, 80), width=120, height=40, text="Apply", color=(220, 220, 250))

        # Joystick and DPad below spinboxes
        self.joystick = Joystick(center=(320, 320), radius=100, knob_radius=35)
        self.dpad = DPad(center=(660, 320), outer_radius=100, inner_radius=25)

        # Buttons on left
        self.buttons = {
            "Guided": Button(position=(30, 100), width=130, height=45, text="Guided", color=(255, 223, 186)),
            "Arm": Button(position=(30, 155), width=130, height=45, text="Arm", color=(186, 225, 255)),
            "Takeoff": Button(position=(30, 210), width=130, height=45, text="Takeoff", color=(204, 255, 204)),
            "Land": Button(position=(30, 265), width=130, height=45, text="Land", color=(255, 204, 203)),
        }

        # Velocity output labels at bottom
        self.labels = {
            "Vel X": TextLabel(position=(200, 450), width=250, height=40, text="X Velocity: 0.00", align="center", font=self.big_font),
            "Vel Y": TextLabel(position=(530, 450), width=250, height=40, text="Y Velocity: 0.00", align="center", font=self.big_font),
        }

        self.max_joystick_value = 1
        self.dpad_strength = 1

    def run(self):
        while self.running:
            self.screen.fill((240, 240, 240))

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                self.joystick.handle_event(event)
                self.dpad.handle_event(event)
                self.apply_button.handle_event(event)
                for btn in self.buttons.values():
                    btn.handle_event(event)
                for box in self.spinboxes.values():
                    box.handle_event(event)

            if self.apply_button.is_active():
                self.max_joystick_value = self.spinboxes["Joystick Max"].get_value()
                self.dpad_strength = self.spinboxes["DPad Value"].get_value()

            # Joystick values
            jx, jy = self.joystick.get_value()
            x_val = jx * self.max_joystick_value
            y_val = -jy * self.max_joystick_value  # Inverted Y-axis (up is positive)

            # DPad direction -> height/yaw
            height_val = yaw_val = 0.0
            direction = self.dpad.get_active_direction()
            if direction == "UP":
                height_val = self.dpad_strength
            elif direction == "DOWN":
                height_val = -self.dpad_strength
            elif direction == "LEFT":
                yaw_val = -self.dpad_strength
            elif direction == "RIGHT":
                yaw_val = self.dpad_strength

            guided = float(self.buttons["Guided"].is_active())
            arm = float(self.buttons["Arm"].is_active())
            takeoff = float(self.buttons["Takeoff"].is_active())
            land = float(self.buttons["Land"].is_active())

            # Update labels
            self.labels["Vel X"].set_text(f"X Velocity: {x_val:.2f}")
            self.labels["Vel Y"].set_text(f"Y Velocity: {y_val:.2f}")

            # Draw everything
            self.joystick.draw(self.screen)
            self.dpad.draw(self.screen)
            for btn in self.buttons.values():
                btn.draw(self.screen)
            for lbl in self.labels.values():
                lbl.draw(self.screen)
            for box in self.spinboxes.values():
                box.draw(self.screen)
            self.apply_button.draw(self.screen)

            # ROS Publishing
            msg = Float32MultiArray()
            msg.data = [float(v) for v in [x_val, y_val, height_val, yaw_val, guided, arm, takeoff, land]]
            self.publisher_.publish(msg)

            pygame.display.flip()
            self.clock.tick(60)
            rclpy.spin_once(self, timeout_sec=0.001)

        pygame.quit()
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    ui = ControllerDrone()
    ui.run()
if __name__ == "__main__":
    main()