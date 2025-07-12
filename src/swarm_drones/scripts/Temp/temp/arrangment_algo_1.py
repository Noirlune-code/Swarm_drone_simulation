import pygame
import sys

# Initialize pygame
pygame.init()

# Constants
GRID_SIZE = 5
CELL_SIZE = 100
PADDING = 10
GAP = 10
FONT_SIZE = 36

WIDTH = HEIGHT = GRID_SIZE * (CELL_SIZE + GAP) + GAP

# Colors
WHITE = (255, 255, 255)
GRAY = (200, 200, 200)
BLUE = (0, 102, 204)
BLACK = (0, 0, 0)
BOX_BORDER_COLOR = BLACK

# Setup screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("5x5 Grid Boxes")

font = pygame.font.SysFont(None, FONT_SIZE)

# Box positions: box_number -> (row, col)
boxes = {
    1: (0, 0),
    2: (0, 1),
    3: (0, 2),
    4: (0, 3),
    5: (0, 4)
}

def draw_grid():
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            x = GAP + col * (CELL_SIZE + GAP)
            y = GAP + row * (CELL_SIZE + GAP)
            rect = pygame.Rect(x, y, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, GRAY, rect, 1)

def draw_boxes():
    for number, (row, col) in boxes.items():
        x = GAP + col * (CELL_SIZE + GAP) + PADDING
        y = GAP + row * (CELL_SIZE + GAP) + PADDING
        size = CELL_SIZE - 2 * PADDING
        box_rect = pygame.Rect(x, y, size, size)

        pygame.draw.rect(screen, BLUE, box_rect)
        pygame.draw.rect(screen, BOX_BORDER_COLOR, box_rect, 3)

        text = font.render(str(number), True, WHITE)
        text_rect = text.get_rect(center=box_rect.center)
        screen.blit(text, text_rect)

def move_box(command):
    if len(command) != 3 or not command.isdigit():
        print(f"Invalid input '{command}'. Must be a 3-digit number.")
        return
    x, y, z = int(command[0]), int(command[1]), int(command[2])
    if x not in boxes:
        print(f"Invalid box number '{x}'.")
        return
    if not (1 <= y <= GRID_SIZE and 1 <= z <= GRID_SIZE):
        print(f"Invalid target coordinates ({y},{z}). Use values from 1 to 5.")
        return
    boxes[x] = (y - 1, z - 1)


# Main game loop
clock = pygame.time.Clock()
running = True

while running:
    screen.fill(WHITE)
    draw_grid()
    draw_boxes()
    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    clock.tick(30)

pygame.quit()
sys.exit()

