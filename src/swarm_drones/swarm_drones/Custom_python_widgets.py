"""
Button Class – Modular Reusable Pygame Button Widget (with Single vs Continuous Mode)

This class defines a flexible and fully customizable button for Pygame GUIs.
It supports label text, press detection in two distinct modes (single tap and continuous hold),
and allows total control over color, font, borders, and layout.

────────────────────────────────────────────────────────────
FEATURES:
────────────────────────────────────────────────────────────
- Configurable dimensions, position, and colors
- Optional automatic darker color when pressed
- Border styling and text customization
- Supports two press behaviors:
  └── continuous_mode = True → keeps returning True while held
  └── continuous_mode = False → only returns True once per tap
- Easily reusable in toolbars, control panels, games, etc.

────────────────────────────────────────────────────────────
USAGE:
────────────────────────────────────────────────────────────
# Create a Button:
button = Button(
    width=120,
    height=60,
    color=(186, 225, 255),
    position=(400, 300),
    text="Arm",
    text_color=(0, 0, 0),
    border_color=(0, 0, 0),
    continuous_mode=False  # False = one-shot, True = held output
)

# Inside game loop:
for event in pygame.event.get():
    button.handle_event(event)

button.draw(screen)

if button.is_active():
    print("Action triggered!")

────────────────────────────────────────────────────────────
METHODS:
────────────────────────────────────────────────────────────
- draw(surface: pygame.Surface)
    Draws the button to the screen.

- handle_event(event: pygame.event.Event)
    Must be called each frame for click/touch detection.

- is_active() -> bool
    Based on mode:
    └── If continuous_mode: returns True while held
    └── Else: returns True once per press

- set_position((x, y))
    Allows dynamic repositioning.

────────────────────────────────────────────────────────────
"""
"""
DPad Class – Modular Pygame Directional Pad (UP, DOWN, LEFT, RIGHT)

This class defines a circular directional pad divided into 4 sectors.
It detects directional presses using mouse input and provides a toggle
for continuous vs one-time (single) output behavior.

────────────────────────────────────────────────────────────
FEATURES:
────────────────────────────────────────────────────────────
- Fully configurable size, position, and colors
- Optional press color (auto darkens base color if not given)
- Optional continuous_mode toggle (continuous vs one-shot output)
- Built-in arrow (triangle) indicator
- Lightweight and reusable in any Pygame project

────────────────────────────────────────────────────────────
USAGE EXAMPLE:
────────────────────────────────────────────────────────────
dpad = DPad(
    center=(500, 300),
    outer_radius=120,
    inner_radius=30,
    base_color=(180, 220, 230),
    border_color=(0, 0, 0),
    triangle_color=(40, 40, 40),
    continuous_mode=False
)

Inside your loop:
    for event in pygame.event.get():
        dpad.handle_event(event)

    dpad.draw(screen)
    direction = dpad.get_active_direction()

────────────────────────────────────────────────────────────
"""
"""
Joystick Class – Modular Pygame Joystick Widget

This class provides a reusable and customizable virtual joystick widget
for any Pygame-based GUI project. It allows smooth analog input via
mouse dragging with support for spring-back, deadzone filtering, and
visual feedback while dragging.

────────────────────────────────────────────────────────────
FEATURES:
────────────────────────────────────────────────────────────
- Fully customizable position, size, and color parameters
- Normalized output in range [-1.0, 1.0] for precise control
- Auto return-to-center (spring-back) behavior for analog feel
- Deadzone support to ignore small unintended movements
- Optional knob clamping to keep motion within the joystick base
- Separate border color options for both base and knob
- Visual touch feedback (active colors) when joystick is held
- Automatically darkens base/knob for active state if not set
- Lightweight, reusable, and easy to integrate into any layout

────────────────────────────────────────────────────────────
CONSTRUCTOR:
────────────────────────────────────────────────────────────
Joystick(
    center: tuple[int, int],                  # (x, y) screen center of base circle
    radius: int,                              # Radius of base circle
    knob_radius: int,                         # Radius of draggable knob
    base_color: tuple[int, int, int] = ...,   # Color of joystick base
    knob_color: tuple[int, int, int] = ...,   # Color of knob
    outer_border_color: tuple[int, int, int] = (0, 0, 0),  # Border color for base
    knob_border_color: tuple[int, int, int] = (0, 0, 0),   # Border color for knob
    active_base_color: tuple[int, int, int] = None,        # Optional: color when base is held
    active_knob_color: tuple[int, int, int] = None,        # Optional: color when knob is held
    spring_back: bool = True,                 # If True, knob resets to center on release
    deadzone: float = 0.05,                   # Threshold below which output is (0, 0)
    lock_to_circle: bool = True               # Prevent knob from moving outside the base circle
)

────────────────────────────────────────────────────────────
METHODS:
────────────────────────────────────────────────────────────
- draw(surface: pygame.Surface)
    Draws the joystick on the provided Pygame surface.

- handle_event(event: pygame.event.Event)
    Handles mouse events (click, release, drag). Should be called
    once per event in the main loop.

- get_value() -> tuple[float, float]
    Returns the normalized output vector (x, y) from -1.0 to 1.0.

- get_knob_position() -> tuple[float, float]
    Returns the absolute (x, y) screen position of the knob.

- reset()
    Resets the knob to its center and output value to (0.0, 0.0).

────────────────────────────────────────────────────────────
TYPICAL USAGE:
────────────────────────────────────────────────────────────

# 1. Create an instance of the joystick
joystick = Joystick(
    center=(300, 200),
    radius=100,
    knob_radius=40,
    base_color=(180, 220, 250),
    knob_color=(70, 130, 180),
    outer_border_color=(0, 0, 0),
    knob_border_color=(255, 0, 0),
    spring_back=True,
    deadzone=0.03,
    lock_to_circle=True
    # active_base_color and active_knob_color are optional
)

# 2. In your game loop

for event in pygame.event.get():
    joystick.handle_event(event)

joystick.draw(screen)

x_val, y_val = joystick.get_value()
# Use x_val and y_val in your logic (e.g., velocity, control)

────────────────────────────────────────────────────────────
"""
"""
SpinBox (NumberStepper) – Modular Numeric Widget for Pygame GUIs

This class implements a horizontal numeric stepper widget, also called a SpinBox.
It allows the user to increment or decrement a number using '+' and '-' buttons.
The value is displayed between the buttons and is constrained within optional
min/max bounds.

────────────────────────────────────────────────────────────
FEATURES:
────────────────────────────────────────────────────────────
- Horizontal layout: [ - ] [value] [ + ]
- Optional min/max constraints
- Configurable step size (e.g. 1, 5, 10)
- Fully stylable: background, text, arrows, border
- Modular and reusable

────────────────────────────────────────────────────────────
USAGE:
────────────────────────────────────────────────────────────
spinbox = SpinBox(
    position=(300, 200),
    width=180,
    height=60,
    value=5,
    step=1,
    min_value=0,
    max_value=10,
    font=pygame.font.SysFont("arial", 28),
    text_color=(0, 0, 0),
    bg_color=(240, 240, 240),
    border_color=(0, 0, 0),
    arrow_color=(80, 80, 80)
)

# In game loop:
for event in pygame.event.get():
    spinbox.handle_event(event)

spinbox.draw(screen)
print("Current value:", spinbox.get_value())

────────────────────────────────────────────────────────────
METHODS:
────────────────────────────────────────────────────────────
- draw(surface)
- handle_event(event)
- get_value()
- set_value(new_value)

────────────────────────────────────────────────────────────
"""
"""
TextLabel – Modular Text Display Widget for Pygame GUIs

This class implements a flexible and reusable label widget that displays static or dynamic text
in a rectangular area. It supports configurable font, alignment, color, and background, making it
suitable for headers, status info, control labels, or UI titles.

────────────────────────────────────────────────────────────
FEATURES:
────────────────────────────────────────────────────────────
- Adjustable position and width/height
- Font, size, color, background, and border customization
- Text alignment: left, center, right
- Optional padding
- Multi-line wrapping support (auto-break if needed – future)
- Dynamically updatable using set_text()

────────────────────────────────────────────────────────────
USAGE:
────────────────────────────────────────────────────────────
label = TextLabel(
    position=(100, 100),
    width=300,
    height=50,
    text="Control Panel",
    font=pygame.font.SysFont("arial", 28),
    text_color=(0, 0, 0),
    bg_color=(220, 220, 220),
    border_color=(0, 0, 0),
    align="center"
)

# In draw loop:
label.draw(screen)

# To update text dynamically:
label.set_text("New Status: OK")

────────────────────────────────────────────────────────────
METHODS:
────────────────────────────────────────────────────────────
- draw(surface)
- set_text(new_text)

────────────────────────────────────────────────────────────
"""

import pygame
import math

class Button:
    def __init__(
        self,
        width,
        height,
        color,
        position=(0, 0),
        text="",
        font=None,
        text_color=(0, 0, 0),
        pressed_color=None,
        border_color=(0, 0, 0),
        border_radius=10,
        continuous_mode=True
    ):
        self.width = width
        self.height = height
        self.color = color
        self.text = text
        self.font = font or pygame.font.SysFont("arial", 24)
        self.text_color = text_color
        self.pressed_color = pressed_color or self._darken_color(color)
        self.border_color = border_color
        self.border_radius = border_radius
        self.position = position
        self.continuous_mode = continuous_mode

        self.rect = pygame.Rect(position[0], position[1], width, height)
        self._mouse_down = False
        self._just_pressed = False
        self._is_active = False

    def _darken_color(self, color, factor=0.85):
        return tuple(max(0, min(255, int(c * factor))) for c in color)

    def set_position(self, position):
        self.position = position
        self.rect.topleft = position

    def draw(self, surface):
        bg_color = self.pressed_color if self._mouse_down else self.color
        pygame.draw.rect(surface, bg_color, self.rect, border_radius=self.border_radius)
        pygame.draw.rect(surface, self.border_color, self.rect, width=2, border_radius=self.border_radius)

        if self.text:
            label = self.font.render(self.text, True, self.text_color)
            surface.blit(label, label.get_rect(center=self.rect.center))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1 and self.rect.collidepoint(event.pos):
                self._mouse_down = True
                self._just_pressed = True
                self._is_active = True

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self._mouse_down = False
                self._is_active = False

    def is_active(self):
        if self.continuous_mode:
            return self._mouse_down and self.rect.collidepoint(pygame.mouse.get_pos())
        else:
            if self._just_pressed:
                self._just_pressed = False
                return True
            return False

class DPad:
    def __init__(
        self,
        center,
        outer_radius,
        inner_radius,
        base_color=(180, 220, 230),
        press_color=None,
        border_color=(0, 0, 0),
        triangle_color=(40, 40, 40),
        continuous_mode=True
    ):
        self.center = center
        self.outer_radius = outer_radius
        self.inner_radius = inner_radius
        self.base_color = base_color
        self.press_color = press_color or self._darken_color(base_color)
        self.border_color = border_color
        self.triangle_color = triangle_color
        self.continuous_mode = continuous_mode

        self._direction = None
        self._mouse_held = False
        self._already_triggered = False

    def _darken_color(self, color, factor=0.85):
        return tuple(max(0, min(255, int(c * factor))) for c in color)

    def draw(self, surface):
        angle_ranges = {
            "UP": (45, 135),
            "RIGHT": (-45, 45),
            "DOWN": (-135, -45),
            "LEFT": (135, -135)
        }

        for direction, angle_range in angle_ranges.items():
            color = self.press_color if self._direction == direction else self.base_color
            self._draw_segment(surface, angle_range, color)
            self._draw_arrow(surface, direction)

        # Draw borders
        for angle in [-45, 45, 135, -135]:
            rad = math.radians(angle)
            x_outer = self.center[0] + self.outer_radius * math.cos(rad)
            y_outer = self.center[1] - self.outer_radius * math.sin(rad)
            pygame.draw.aaline(surface, self.border_color, self.center, (x_outer, y_outer))

        pygame.draw.circle(surface, self.border_color, self.center, self.outer_radius, 3)
        pygame.draw.circle(surface, self.base_color, self.center, self.inner_radius)

    def _draw_segment(self, surface, angle_range, color):
        start, end = angle_range
        points = []

        if start > end:
            outer_angles = list(range(start, 181)) + list(range(-180, end + 1))
        else:
            outer_angles = list(range(start, end + 1))

        for angle in outer_angles:
            rad = math.radians(angle)
            points.append((
                self.center[0] + self.outer_radius * math.cos(rad),
                self.center[1] - self.outer_radius * math.sin(rad)
            ))

        if start > end:
            inner_angles = list(range(end, -181, -1)) + list(range(180, start - 1, -1))
        else:
            inner_angles = list(range(end, start - 1, -1))

        for angle in inner_angles:
            rad = math.radians(angle)
            points.append((
                self.center[0] + self.inner_radius * math.cos(rad),
                self.center[1] - self.inner_radius * math.sin(rad)
            ))

        if len(points) >= 3:
            pygame.draw.polygon(surface, color, points)

    def _draw_arrow(self, surface, direction):
        offset = self.outer_radius - 25
        size = 16
        cx, cy = self.center

        angle_map = {
            "UP": 90,
            "DOWN": -90,
            "LEFT": 180,
            "RIGHT": 0
        }

        angle = angle_map[direction]
        tip = (
            cx + offset * math.cos(math.radians(angle)),
            cy - offset * math.sin(math.radians(angle))
        )

        if direction in ["UP", "DOWN"]:
            base1 = (tip[0] - size, tip[1] + size if direction == "UP" else tip[1] - size)
            base2 = (tip[0] + size, tip[1] + size if direction == "UP" else tip[1] - size)
        else:
            base1 = (tip[0] + size if direction == "LEFT" else tip[0] - size, tip[1] - size)
            base2 = (tip[0] + size if direction == "LEFT" else tip[0] - size, tip[1] + size)

        pygame.draw.polygon(surface, self.triangle_color, [tip, base1, base2])

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                self._mouse_held = True
                direction = self._detect_direction(event.pos)
                if direction:
                    self._direction = direction
                    self._already_triggered = False

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self._mouse_held = False
                self._direction = None
                self._already_triggered = False

        elif event.type == pygame.MOUSEMOTION:
            if self._mouse_held:
                direction = self._detect_direction(event.pos)
                self._direction = direction
                if not self.continuous_mode and self._already_triggered:
                    self._direction = None

    def _detect_direction(self, pos):
        dx = pos[0] - self.center[0]
        dy = self.center[1] - pos[1]  # Y is inverted in screen
        distance = math.hypot(dx, dy)

        if distance < self.inner_radius or distance > self.outer_radius:
            return None

        angle = (math.degrees(math.atan2(dy, dx)) + 360) % 360
        if 45 <= angle < 135:
            return "UP"
        elif 135 <= angle < 225:
            return "LEFT"
        elif 225 <= angle < 315:
            return "DOWN"
        else:
            return "RIGHT"

    def get_active_direction(self):
        """Returns current active direction (or None) based on mode"""
        if self._direction:
            if self.continuous_mode:
                return self._direction
            else:
                if not self._already_triggered:
                    self._already_triggered = True
                    return self._direction
        return None

class Joystick:
    def __init__(
        self,
        center,
        radius,
        knob_radius,
        base_color=(150, 150, 150),
        knob_color=(50, 50, 50),
        outer_border_color=(0, 0, 0),
        knob_border_color=(0, 0, 0),
        active_base_color=None,
        active_knob_color=None,
        spring_back=True,
        deadzone=0.05,
        lock_to_circle=True
    ):
        self.center = center
        self.radius = radius
        self.knob_radius = knob_radius

        self.base_color = base_color
        self.knob_color = knob_color
        self.outer_border_color = outer_border_color
        self.knob_border_color = knob_border_color

        self.active_base_color = active_base_color or self._darken_color(base_color)
        self.active_knob_color = active_knob_color or self._darken_color(knob_color)

        self.spring_back = spring_back
        self.deadzone = deadzone
        self.lock_to_circle = lock_to_circle

        self.knob_pos = self.center
        self._dragging = False
        self._value = (0.0, 0.0)

    def _darken_color(self, color, factor=0.85):
        """Return a slightly darker version of the given RGB color."""
        return tuple(max(0, min(255, int(c * factor))) for c in color)

    def draw(self, surface):
        """Draws the joystick base and knob on the screen."""
        base_col = self.active_base_color if self._dragging else self.base_color
        knob_col = self.active_knob_color if self._dragging else self.knob_color

        pygame.draw.circle(surface, base_col, self.center, self.radius)
        pygame.draw.circle(surface, self.outer_border_color, self.center, self.radius, width=3)

        pygame.draw.circle(surface, knob_col, self.knob_pos, self.knob_radius)
        pygame.draw.circle(surface, self.knob_border_color, self.knob_pos, self.knob_radius, width=2)

    def handle_event(self, event):
        """Handles mouse events for interaction."""
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self._within_knob(event.pos):
                self._dragging = True

        elif event.type == pygame.MOUSEBUTTONUP:
            self._dragging = False
            if self.spring_back:
                self.knob_pos = self.center
                self._value = (0.0, 0.0)

        elif event.type == pygame.MOUSEMOTION and self._dragging:
            self._update_knob_position(event.pos)

    def _within_knob(self, pos):
        dx = pos[0] - self.knob_pos[0]
        dy = pos[1] - self.knob_pos[1]
        return math.hypot(dx, dy) <= self.knob_radius

    def _update_knob_position(self, pos):
        dx = pos[0] - self.center[0]
        dy = pos[1] - self.center[1]
        dist = math.hypot(dx, dy)

        if self.lock_to_circle and dist > self.radius:
            angle = math.atan2(dy, dx)
            dx = self.radius * math.cos(angle)
            dy = self.radius * math.sin(angle)

        knob_x = self.center[0] + dx
        knob_y = self.center[1] + dy
        self.knob_pos = (knob_x, knob_y)

        norm_x = dx / self.radius
        norm_y = dy / self.radius

        if math.hypot(norm_x, norm_y) < self.deadzone:
            norm_x, norm_y = 0.0, 0.0

        self._value = (norm_x, norm_y)

    def get_value(self):
        """Returns the normalized joystick vector (x, y)."""
        return self._value

    def get_knob_position(self):
        """Returns the knob position in screen coordinates."""
        return self.knob_pos

    def reset(self):
        """Manually reset the knob to center."""
        self.knob_pos = self.center
        self._value = (0.0, 0.0)

class SpinBox:
    def __init__(
        self,
        position,
        width,
        height,
        value=0,
        step=1,
        min_value=None,
        max_value=None,
        font=None,
        text_color=(0, 0, 0),
        bg_color=(230, 230, 230),
        border_color=(0, 0, 0),
        arrow_color=(50, 50, 50)
    ):
        self.x, self.y = position
        self.width = width
        self.height = height
        self.value = value
        self.step = step
        self.min_value = min_value
        self.max_value = max_value

        self.font = font or pygame.font.SysFont("arial", 24)
        self.text_color = text_color
        self.bg_color = bg_color
        self.border_color = border_color
        self.arrow_color = arrow_color

        self.box_rect = pygame.Rect(self.x, self.y, self.width, self.height)

        # Define zones for [-], value, [+]
        self.button_width = self.height  # Make buttons square
        self.minus_rect = pygame.Rect(self.x, self.y, self.button_width, self.height)
        self.plus_rect = pygame.Rect(self.x + self.width - self.button_width, self.y, self.button_width, self.height)
        self.value_rect = pygame.Rect(self.minus_rect.right, self.y, self.width - 2 * self.button_width, self.height)

    def draw(self, surface):
        # Draw entire box
        pygame.draw.rect(surface, self.bg_color, self.box_rect)
        pygame.draw.rect(surface, self.border_color, self.box_rect, 2)

        # Draw minus button
        pygame.draw.rect(surface, self.arrow_color, self.minus_rect)
        minus_label = self.font.render("-", True, (255, 255, 255))
        surface.blit(minus_label, minus_label.get_rect(center=self.minus_rect.center))

        # Draw plus button
        pygame.draw.rect(surface, self.arrow_color, self.plus_rect)
        plus_label = self.font.render("+", True, (255, 255, 255))
        surface.blit(plus_label, plus_label.get_rect(center=self.plus_rect.center))

        # Draw value
        value_surf = self.font.render(str(self.value), True, self.text_color)
        surface.blit(value_surf, value_surf.get_rect(center=self.value_rect.center))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.minus_rect.collidepoint(event.pos):
                self._decrement()
            elif self.plus_rect.collidepoint(event.pos):
                self._increment()

    def _increment(self):
        new_val = self.value + self.step
        if self.max_value is None or new_val <= self.max_value:
            self.value = new_val

    def _decrement(self):
        new_val = self.value - self.step
        if self.min_value is None or new_val >= self.min_value:
            self.value = new_val

    def get_value(self):
        return self.value

    def set_value(self, new_value):
        if self.min_value is not None and new_value < self.min_value:
            return
        if self.max_value is not None and new_value > self.max_value:
            return
        self.value = new_value

class TextLabel:
    def __init__(
        self,
        position,
        width,
        height,
        text,
        font=None,
        text_color=(0, 0, 0),
        bg_color=(245, 245, 245),
        border_color=(0, 0, 0),
        align="left",
        padding=10
    ):
        self.x, self.y = position
        self.width = width
        self.height = height
        self.text = text
        self.font = font or pygame.font.SysFont("arial", 24)
        self.text_color = text_color
        self.bg_color = bg_color
        self.border_color = border_color
        self.align = align  # "left", "center", "right"
        self.padding = padding
        self.rect = pygame.Rect(self.x, self.y, self.width, self.height)

    def draw(self, surface):
        # Draw background and border
        pygame.draw.rect(surface, self.bg_color, self.rect)
        pygame.draw.rect(surface, self.border_color, self.rect, 2)

        # Render text
        text_surf = self.font.render(self.text, True, self.text_color)
        text_rect = text_surf.get_rect()

        if self.align == "left":
            text_rect.midleft = (self.x + self.padding, self.y + self.height // 2)
        elif self.align == "center":
            text_rect.center = (self.x + self.width // 2, self.y + self.height // 2)
        elif self.align == "right":
            text_rect.midright = (self.x + self.width - self.padding, self.y + self.height // 2)

        surface.blit(text_surf, text_rect)

    def set_text(self, new_text):
        self.text = new_text



