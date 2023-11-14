import pygame as pg
import math
from common.math import *
from common.shape import *
from physic_engine.body import *


class Renderer:
    def __init__(self, world, width, height):
        pg.init()
        self.world = world
        self.width = width
        self.height = height
        self.screen = pg.display.set_mode((width, height))
        pg.display.set_caption("2D physic engine")

    def draw_objects(self):
        self.screen.fill((0, 0, 0))
        bodies = self.world.get_bodies()
        for body in bodies:
            body.shape.Draw(self.screen)

    def renderControlPanel(self):
        font = pg.font.Font(None, 20)
        text = font.render("text", True, (255, 255, 255))
        self.screen.blit(text, (0, 0))

    def render_frame(self):
        self.draw_objects()
        if self.world.debugMode:
            self.renderControlPanel()
        pg.display.flip()
