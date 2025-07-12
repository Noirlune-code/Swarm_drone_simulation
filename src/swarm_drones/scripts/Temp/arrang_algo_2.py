import pygame
import sys

# === CONFIGURATION === #
GRID_SIZE = 5
CELL_SIZE = 80
GAP = 10
FONT_SIZE = 24
BUTTON_HEIGHT = 40
MAX_SELECTIONS = 5

# Colors
WHITE = (255, 255, 255)
GRAY = (210, 210, 210)
DARK_GRAY = (130, 130, 130)
BLUE = (50, 100, 255)
GREEN = (50, 200, 50)
RED = (200, 50, 50)
YELLOW = (255, 255, 100)
BLACK = (0, 0, 0)
ORANGE = (255, 165, 0)

# === INITIALIZE === #
pygame.init()
FONT = pygame.font.SysFont(None, FONT_SIZE)
WIDTH = 2 * (GRID_SIZE * (CELL_SIZE + GAP)) + GAP
HEIGHT = GRID_SIZE * (CELL_SIZE + GAP) + GAP + BUTTON_HEIGHT + 30
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Drone Grid Simulator")

# === GRIDS === #
# Drone positions: (row, col) => drone_id
drones = {i + 1: (0, i) for i in range(5)}
locked_drones = set()

# Input grid: set of selected (row, col) tuples
input_positions = []

# Movement queue: list of (drone_id, target_row, target_col)
movement_queue = []
intermediate_movements = []

movement_index = 0
config_locked = False

# === BUTTONS === #
def create_button(x, y, w, h, text, color):
    return {"rect": pygame.Rect(x, y, w, h), "text": text, "color": color}

buttons = [
    create_button(WIDTH // 2 - 210, HEIGHT - 50, 120, BUTTON_HEIGHT, "Reset", RED),
    create_button(WIDTH // 2 - 60, HEIGHT - 50, 160, BUTTON_HEIGHT, "Lock Configuration", BLUE),
    create_button(WIDTH // 2 + 120, HEIGHT - 50, 160, BUTTON_HEIGHT, "Stepwise Movement", GREEN),
]

# === FUNCTIONS === #
def draw_grid(grid_x_offset, grid_data, is_drone_grid):
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            x = grid_x_offset + GAP + col * (CELL_SIZE + GAP)
            y = GAP + row * (CELL_SIZE + GAP)
            rect = pygame.Rect(x, y, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, GRAY, rect)

            # Drone Grid
            if is_drone_grid:
                for drone_id, (dr, dc) in drones.items():
                    if (dr, dc) == (row, col):
                        if drone_id in locked_drones:
                            color = GREEN
                        elif (dr, dc) in [m[1] for m in intermediate_movements]:
                            color = ORANGE
                        else:
                            color = BLUE
                        pygame.draw.rect(screen, color, rect.inflate(-10, -10))
                        text = FONT.render(str(drone_id), True, WHITE)
                        text_rect = text.get_rect(center=rect.center)
                        screen.blit(text, text_rect)
            else:
                if (row, col) in grid_data:
                    pygame.draw.rect(screen, YELLOW, rect.inflate(-10, -10))

            pygame.draw.rect(screen, BLACK, rect, 2)

def draw_buttons():
    for btn in buttons:
        pygame.draw.rect(screen, btn["color"], btn["rect"])
        text = FONT.render(btn["text"], True, WHITE)
        text_rect = text.get_rect(center=btn["rect"].center)
        screen.blit(text, text_rect)

def pos_in_grid(mouse_pos, grid_x_offset):
    mx, my = mouse_pos
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            x = grid_x_offset + GAP + col * (CELL_SIZE + GAP)
            y = GAP + row * (CELL_SIZE + GAP)
            rect = pygame.Rect(x, y, CELL_SIZE, CELL_SIZE)
            if rect.collidepoint(mx, my):
                return (row, col)
    return None

def handle_input_click(pos):
    global input_positions
    if config_locked:
        return
    if pos in input_positions:
        input_positions.remove(pos)
    elif len(input_positions) < MAX_SELECTIONS:
        input_positions.append(pos)

def handle_button_click(mouse_pos):
    global input_positions, config_locked, movement_queue, movement_index, intermediate_movements

    for i, btn in enumerate(buttons):
        if btn["rect"].collidepoint(mouse_pos):
            if i == 0:  # Reset
                input_positions = []
                config_locked = False
                movement_queue = []
                intermediate_movements = []
                movement_index = 0
            elif i == 1 and not config_locked:  # Lock Configuration
                config_locked = True
                prepare_movement_queue()
            elif i == 2 and config_locked:  # Stepwise Movement
                execute_next_move()

def prepare_movement_queue():
    global movement_queue
    sorted_positions = sorted(input_positions, key=lambda pos: (-pos[0], pos[1]))  # Row 4→0, Col 0→4
    assigned_drones = set()
    for row, col in sorted_positions:
        movement_queue.append((None, row, col))  # Drone assignment during execution

def execute_next_move():
    global movement_index, movement_queue, intermediate_movements

    if movement_index >= len(movement_queue):
        return

    current = movement_queue[movement_index]
    drone_id, target_row, target_col = current

    if drone_id is None:
        # Assign drone to this move
        if any(drones[d] == (0, target_col) for d in drones if d not in locked_drones):
            # Drone already at top of target column
            for d in sorted(drones):
                if drones[d] == (0, target_col) and d not in locked_drones:
                    movement_queue[movement_index] = (d, target_row, target_col)
                    break
        else:
            # Move a nearby drone to (0, target_col)
            for d in sorted(drones):
                if d not in locked_drones and drones[d][0] == 0:
                    drones[d] = (0, target_col)
                    intermediate_movements.append((d, (0, target_col)))
                    return  # Wait until next press
    else:
        # Move assigned drone to target
        drones[drone_id] = (target_row, target_col)
        locked_drones.add(drone_id)
        movement_index += 1
        intermediate_movements = [m for m in intermediate_movements if m[0] != drone_id]

# === MAIN LOOP === #
clock = pygame.time.Clock()
running = True

while running:
    screen.fill(WHITE)
    draw_grid(GAP, input_positions, is_drone_grid=False)
    draw_grid(WIDTH // 2 + GAP, input_positions, is_drone_grid=True)
    draw_buttons()

    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                if event.pos[1] < HEIGHT - 60:
                    # Left grid click
                    pos = pos_in_grid(event.pos, GAP)
                    if pos:
                        handle_input_click(pos)
                    # Right grid is not clickable
                handle_button_click(event.pos)

    clock.tick(30)

pygame.quit()
sys.exit()
