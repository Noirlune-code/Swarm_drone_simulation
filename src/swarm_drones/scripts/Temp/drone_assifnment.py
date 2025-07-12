import pygame
import sys
import time

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

# === BUTTON HELPERS === #
def create_button(x, y, w, h, text, color):
    return {"rect": pygame.Rect(x, y, w, h), "text": text, "color": color}

def draw_button(surface, btn):
    pygame.draw.rect(surface, btn["color"], btn["rect"])
    text = FONT.render(btn["text"], True, WHITE)
    text_rect = text.get_rect(center=btn["rect"].center)
    surface.blit(text, text_rect)


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
                        commands.append((int(drone_id[1]), x, y))
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
                        commands.append((int(nearest_drone[1]), x, 0))
                        commands.append((int(nearest_drone[1]), x, y))
        return commands


# === VISUALIZER === #
class DroneVisualizer:
    def __init__(self, grid_size, x_offset, y_offset):
        self.grid_size = grid_size
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.positions = {
            "D1": (0, 0),
            "D2": (0, 1),
            "D3": (0, 2),
            "D4": (0, 3),
            "D5": (0, 4),
        }
        self.commands = []
        self.step_index = 0
        self.auto_play = False
        self.last_step_time = pygame.time.get_ticks()

    def load_commands(self, commands):
        self.commands = commands
        self.step_index = 0
        self.auto_play = False
        self.positions = {
            "D1": (0, 0),
            "D2": (0, 1),
            "D3": (0, 2),
            "D4": (0, 3),
            "D5": (0, 4),
        }

    def draw(self, surface):
        for drone_id, (x, y) in self.positions.items():
            x_pix = self.x_offset + GAP + x * (CELL_SIZE + GAP)
            y_pix = self.y_offset + GAP + y * (CELL_SIZE + GAP)
            rect = pygame.Rect(x_pix, y_pix, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(surface, ORANGE, rect.inflate(-10, -10))
            text = FONT.render(drone_id, True, BLACK)
            text_rect = text.get_rect(center=rect.center)
            surface.blit(text, text_rect)

    def step(self):
        if self.step_index < len(self.commands):
            drone_id, x, y = self.commands[self.step_index]
            self.positions[drone_id] = (x, y)
            self.step_index += 1

    def update(self):
        if self.auto_play:
            now = pygame.time.get_ticks()
            if now - self.last_step_time >= AUTO_PLAY_INTERVAL_MS:
                self.step()
                self.last_step_time = now

    def reset(self):
        self.load_commands([])

# === MAIN LOOP === #
def main():
    grid_size = 5
    grid_width = grid_size * (CELL_SIZE + GAP) + GAP
    grid_height = grid_size * (CELL_SIZE + GAP) + GAP
    screen_width = grid_width + 2 * GAP
    screen_height = grid_height + 6 * (BUTTON_HEIGHT + 10)

    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Drone Grid Visualizer")

    grid = GridSelector(grid_size, GAP, GAP)
    engine = DroneAssignmentEngine(grid_size)
    visualizer = DroneVisualizer(grid_size, GAP, GAP)

    btn_y_start = grid_height + 20
    buttons = [
        create_button(GAP, btn_y_start, 200, BUTTON_HEIGHT, "Print Coordinates", BLUE),
        create_button(GAP, btn_y_start + 1 * (BUTTON_HEIGHT + 10), 200, BUTTON_HEIGHT, "Generate Commands", ORANGE),
        create_button(GAP, btn_y_start + 2 * (BUTTON_HEIGHT + 10), 200, BUTTON_HEIGHT, "Next Step", GREEN),
        create_button(GAP, btn_y_start + 3 * (BUTTON_HEIGHT + 10), 200, BUTTON_HEIGHT, "Auto Play", GREEN),
        create_button(GAP, btn_y_start + 4 * (BUTTON_HEIGHT + 10), 200, BUTTON_HEIGHT, "Stop Auto Play", RED),
        create_button(GAP, btn_y_start + 5 * (BUTTON_HEIGHT + 10), 200, BUTTON_HEIGHT, "Reset", RED),
    ]

    clock = pygame.time.Clock()
    running = True

    while running:
        screen.fill(WHITE)
        grid.draw(screen)
        visualizer.draw(screen)
        for btn in buttons:
            draw_button(screen, btn)
        pygame.display.flip()

        visualizer.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                for i, btn in enumerate(buttons):
                    if btn["rect"].collidepoint((mx, my)):
                        if i == 0:
                            print("Selected:", grid.get_selected())
                        elif i == 1:
                            cmds = engine.generate_commands(grid.get_selected())
                            visualizer.load_commands(cmds)
                            for c in cmds:
                                print(c)
                        elif i == 2:
                            visualizer.step()
                        elif i == 3:
                            visualizer.auto_play = True
                        elif i == 4:
                            visualizer.auto_play = False
                        elif i == 5:
                            grid.reset()
                            visualizer.reset()
                        break
                else:
                    grid.handle_click((mx, my))

        clock.tick(30)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
