import pygame as pg
from common.math import *
from common.shape import *
from visualization.renderer import Renderer
from game_engine.eventManager import EventManager
from game_engine.world import World
from game_engine.clock import Clock

WIDTH = 800
HEIGHT = 600
FPS = 60

accumulator = 0
dt = 1.0 / FPS
frameStepping = False
canStep = False

Clock = Clock()
World = World(dt, 10)
Renderer = Renderer(World, WIDTH, HEIGHT)
EventManager = EventManager(World)


circle = Circle(20)
World.add(circle, 400, 300)

# Main loop
while EventManager.is_running():
    accumulator += Clock.Elapsed()

    Clock.Start()

    accumulator = Clamp(0.0, 0.1, accumulator)
    while accumulator >= dt:
        if not frameStepping:
            World.step()
        else:
            if canStep:
                World.step()
                canStep = False
        accumulator -= dt

    Clock.Stop()

    EventManager.process_events()
    Renderer.render_frame()

pg.quit()
