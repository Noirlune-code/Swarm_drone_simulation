import pygame

import sys


class GridSelector:
    def __init__(self, grid_size, x_offset, y_offset, cell_size, gap, font,
                 max_selections=5,
                 bg_color=(210, 210, 210), select_color=(255, 255, 100), border_color=(0, 0, 0)):
        self.grid_size = grid_size
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.cell_size = cell_size
        self.gap = gap
        self.font = font
        self.max_selections = max_selections
        self.bg_color = bg_color
        self.select_color = select_color
        self.border_color = border_color
        self.selected = []
        self.locked = False

    def draw(self, surface):
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                x = self.x_offset + self.gap + col * (self.cell_size + self.gap)
                y = self.y_offset + self.gap + row * (self.cell_size + self.gap)
                rect = pygame.Rect(x, y, self.cell_size, self.cell_size)
                pygame.draw.rect(surface, self.bg_color, rect)
                if (col, row) in self.selected:
                    pygame.draw.rect(surface, self.select_color, rect.inflate(-10, -10))
                pygame.draw.rect(surface, self.border_color, rect, 2)

    def handle_click(self, mouse_pos):
        if self.locked:
            return
        mx, my = mouse_pos
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                x = self.x_offset + self.gap + col * (self.cell_size + self.gap)
                y = self.y_offset + self.gap + row * (self.cell_size + self.gap)
                rect = pygame.Rect(x, y, self.cell_size, self.cell_size)
                if rect.collidepoint(mx, my):
                    pos = (col, row)
                    if pos in self.selected:
                        self.selected.remove(pos)
                    elif len(self.selected) < self.max_selections:
                        self.selected.append(pos)
                    return

    def reset(self):
        self.selected.clear()
        self.locked = False

    def lock(self):
        self.locked = True

    def get_selected(self):
        return list(self.selected)

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

class DroneAssignmentEngine:
    def __init__(self, grid_size, start_positions):
        self.grid_size = grid_size
        self.drone_start_positions = start_positions

    def generate_commands(self, selected_cells):
        commands = []
        drone_available = set(self.drone_start_positions.keys())

        selected_by_row = {}
        for x, y in selected_cells:
            selected_by_row.setdefault(y, []).append((x, y))

        for y in sorted(selected_by_row.keys(), reverse=True):
            for x, _ in sorted(selected_by_row[y]):
                assigned = False
                for drone_id in sorted(self.drone_start_positions):
                    if self.drone_start_positions[drone_id] == (0, x) and drone_id in drone_available:
                        commands.append((drone_id, x, y))
                        drone_available.remove(drone_id)
                        assigned = True
                        break

                if not assigned:
                    nearest = min(drone_available,
                                  key=lambda d: abs(self.drone_start_positions[d][1] - x),
                                  default=None)
                    if nearest:
                        commands.append((nearest, x, 0))
                        commands.append((nearest, x, y))
                        drone_available.remove(nearest)

        return commands



def main():
    pygame.init()
    font = pygame.font.SysFont(None, 24)

    grid_size = 5
    cell_size = 80
    gap = 10
    button_height = 40
    panel_margin = 10

    screen_width = grid_size * (cell_size + gap) + 2 * gap
    screen_height = grid_size * (cell_size + gap) + 5 * (button_height + panel_margin)

    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Drone Grid Planner")

    grid = GridSelector(grid_size, gap, gap, cell_size, gap, font)

    print("hi")

    start_positions = {
        "D1": (0, 0),
        "D2": (0, 1),
        "D3": (0, 2),
        "D4": (0, 3),
        "D5": (0, 4)
    }
    engine = DroneAssignmentEngine(grid_size, start_positions)

    buttons = [
        Button(200, button_height, (50, 100, 255), (gap, screen_height - 4 * (button_height + panel_margin)), "Lock Configuration", font, (255, 255, 255)),
        Button(200, button_height, (255, 165, 0), (gap, screen_height - 3 * (button_height + panel_margin)), "Process Configuration", font, (255, 255, 255)),
        Button(200, button_height, (50, 200, 50), (gap, screen_height - 2 * (button_height + panel_margin)), "Make Formation", font, (255, 255, 255)),
        Button(200, button_height, (200, 50, 50), (gap, screen_height - 1 * (button_height + panel_margin)), "Reset", font, (255, 255, 255)),
    ]

    clock = pygame.time.Clock()
    running = True
    while running:
        screen.fill((255, 255, 255))
        grid.draw(screen)
        for btn in buttons:
            btn.draw(screen)
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # grid.handle_click(event.pos)
            # for btn in buttons:
            #     btn.handle_event(event)
            if event.type == pygame.MOUSEBUTTONDOWN:
                grid.handle_click(event.pos)

            for btn in buttons:
                btn.handle_event(event)

        if buttons[0].is_active():
            grid.lock()
            print("Configuration locked.")

        if buttons[1].is_active():
            result = engine.generate_commands(grid.get_selected())
            print("Generated commands:")
            for cmd in result:
                print(cmd)

        if buttons[2].is_active():
            print("Make formation — to be implemented in pymavlink movement class.")

        if buttons[3].is_active():
            grid.reset()
            print("Reset.")

        clock.tick(30)

    pygame.quit()
    # sys.exit()


main()