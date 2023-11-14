import pygame as pg
from common.shape import Circle


class EventManager:
    def __init__(self, world):
        self.world = world
        self.running = True

    def process_events(self):
        for event in pg.event.get():
            if event.type == pg.QUIT or (
                event.type == pg.KEYDOWN and event.key == pg.K_ESCAPE
            ):
                self.running = False
            elif event.type == pg.MOUSEBUTTONDOWN:
                self.handle_mouse_click(event)

    def handle_mouse_click(self, event):
        x, y = event.pos
        new_circle = Circle(20)  # Radius of 20
        self.world.add(new_circle, x, y)

    def is_running(self):
        return self.running
