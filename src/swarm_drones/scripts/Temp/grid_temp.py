import pygame
import sys

# === CONFIGURATION === #
CELL_SIZE = 80
GAP = 10
FONT_SIZE = 24
BUTTON_HEIGHT = 40
MAX_SELECTIONS = 5

# Colors
WHITE = (255, 255, 255)
GRAY = (210, 210, 210)
BLACK = (0, 0, 0)
YELLOW = (255, 255, 100)
BLUE = (50, 100, 255)
GREEN = (50, 200, 50)
RED = (200, 50, 50)

# === INITIALIZE === #
pygame.init()
FONT = pygame.font.SysFont(None, FONT_SIZE)


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

                if (row, col) in self.selected:
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
                    pos = (row, col)
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


# === BUTTON UTILITIES === #
def create_button(x, y, w, h, text, color):
    return {"rect": pygame.Rect(x, y, w, h), "text": text, "color": color}


def draw_button(surface, btn):
    pygame.draw.rect(surface, btn["color"], btn["rect"])
    text = FONT.render(btn["text"], True, WHITE)
    text_rect = text.get_rect(center=btn["rect"].center)
    surface.blit(text, text_rect)


# === DEMO APP === #
def main():
    grid_size = 5  # You can change this to any value like 6, 7, etc.
    grid_width = grid_size * (CELL_SIZE + GAP) + GAP
    grid_height = grid_size * (CELL_SIZE + GAP) + GAP
    window_width = grid_width + 2 * GAP
    window_height = grid_height + BUTTON_HEIGHT * 3 + 60

    screen = pygame.display.set_mode((window_width, window_height))
    pygame.display.set_caption("GridSelector Demo")

    grid = GridSelector(grid_size=grid_size, x_offset=GAP, y_offset=GAP)

    # Buttons
    btn_y_start = grid_height + 30
    buttons = [
        create_button(GAP, btn_y_start, 180, BUTTON_HEIGHT, "Print Coordinates", BLUE),
        create_button(GAP, btn_y_start + BUTTON_HEIGHT + 10, 180, BUTTON_HEIGHT, "Reset", RED),
        create_button(GAP, btn_y_start + 2 * (BUTTON_HEIGHT + 10), 180, BUTTON_HEIGHT, "Lock Grid", GREEN),
    ]

    clock = pygame.time.Clock()
    running = True

    while running:
        screen.fill(WHITE)
        grid.draw(screen)

        for btn in buttons:
            draw_button(screen, btn)

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                # Check buttons
                for i, btn in enumerate(buttons):
                    if btn["rect"].collidepoint((mx, my)):
                        if i == 0:
                            print("Selected coordinates:", grid.get_selected())
                        elif i == 1:
                            grid.reset()
                        elif i == 2:
                            grid.lock()
                        break
                else:
                    # If not a button, maybe it's a grid click
                    grid.handle_click((mx, my))

        clock.tick(30)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
